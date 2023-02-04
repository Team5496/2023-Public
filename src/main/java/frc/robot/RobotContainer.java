// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlanner;
import frc.robot.commands.DefaultDriveCommand;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Lights;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final Joystick m_controller = new Joystick(0);
  private final Lights m_lights = new Lights();
  public void zeromotors() {
    m_drivetrainSubsystem.m_backLeftModule.set(0.0, 0.0);
    m_drivetrainSubsystem.m_backRightModule.set(0.0, 0.0);
    m_drivetrainSubsystem.m_frontLeftModule.set(0.0, 0.0);
    m_drivetrainSubsystem.m_frontRightModule.set(0.0, 0.0);
  }

  public double exponentiate(double x){
    return Math.pow(x, 3);
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
            () -> -modifyAxis((exponentiate(m_controller.getY()))) * 0.5 * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(exponentiate(m_controller.getX())) * 0.5 * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(exponentiate(m_controller.getZ())) * 0.5 * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
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
  public Command getAutonomousCommand(String configuration, int count) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(configuration+"/gotopiece"+configuration, new PathConstraints(4.97, 3));

    switch (count){
      case 0:
        trajectory = PathPlanner.loadPath(configuration+"/gotopiece"+configuration, new PathConstraints(4.97, 3));
      case 1:
        trajectory = PathPlanner.loadPath(configuration+"/placepiece"+configuration, new PathConstraints(4.97, 3));
    }

    Command autocommand = m_drivetrainSubsystem.generatetrajectory(trajectory, true);
    PathPlannerState examplestate = (PathPlannerState) trajectory.sample(0.4);
    

    System.out.println(examplestate.velocityMetersPerSecond);
    return autocommand;
  }

  public Command getSimpleCommand(String config) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("move" + config, new PathConstraints(4, 3));
    Command autocommand = m_drivetrainSubsystem.generatetrajectory(trajectory, true);
    return autocommand;
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
