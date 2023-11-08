// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.IntakeSwivel;
import frc.robot.subsystems.IntakeArm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public final Arm m_arm = new Arm();
  public final Elevator m_elevator = new Elevator();
  public final Intake m_intake = new Intake();
  public final IntakeSwivel m_intakeSwivel = new IntakeSwivel();
  public final IntakeArm m_intakeArm = new IntakeArm();

  private final Joystick m_controller = new Joystick(0);
  private final Lights m_lights = new Lights();

  private SlewRateLimiter accel_limiter = new SlewRateLimiter(1.9);
  private SlewRateLimiter rotate_limiter = new SlewRateLimiter(1.9);
  private SlewRateLimiter clock_limiter = new SlewRateLimiter(1.9);
  
  /* 
  public double exponentiate(double x){
    if (x < 0.50) {
      return x * 0.50;
    } else {
      return Math.pow(x, 3);
    }
  }
  */

  public double setInput(double num) {
    if (m_controller.getTrigger()) { return (num / 2.0); } else return num;
  }

  public double setInputZ(double z) {
    if (m_controller.getTrigger()) {return (z / 2.5);} else return z;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> setInput(modifyAxis(rotate_limiter.calculate(m_controller.getX()))) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> setInput(-modifyAxis(accel_limiter.calculate(m_controller.getY()))) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> setInputZ(-modifyAxis(clock_limiter.calculate(m_controller.getZ()))) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   // new Button(m_controller::getBackButton)
            // No requirements because we don't need to interrupt anything
        //  .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(int count) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("getonepiecebalance/1", new PathConstraints(3, 2));
    
    if (count == 2) {
      trajectory = PathPlanner.loadPath("2", new PathConstraints(3, 2));
    }

    Command autoCommand = m_drivetrainSubsystem.generateTrajectory(trajectory, true);    
    return autoCommand;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
