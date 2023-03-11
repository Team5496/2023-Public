// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import frc.robot.model.EnumToCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Command[] m_autonomousCommands = new Command[2];
  private Command[] m_autonomousArmCommands = new Command[2];
  private Command pollcommand_isfinished;
  private Arm arm = new Arm();
  private Elevator elevator = new Elevator();
  private Intake intake = new Intake();
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private EnumToCommand teleopcommandhandler = new EnumToCommand(elevator, arm, intake);
  private GenericHID controlBoard = new GenericHID(1);
  private XboxController controlIntake = new XboxController(2);

  private Boolean foundapriltag = false;
  private double recordedapriltagdistanceforpath = 0.0;
  private int recordedapriltagID = 0;
  private Limelight limelight = new Limelight("gloworm");

  // TELEOP
  SequentialCommandGroup conePickUpGroup = new SequentialCommandGroup(elevator.getPositionCommand(Constants.ELEVATOR_HIGH), arm.getPositionCommand(Constants.ARM_DOWN), elevator.getPositionCommand(Constants.ELEVATOR_PICK_UP));
  SequentialCommandGroup carryGroup = new SequentialCommandGroup(elevator.getPositionCommand(Constants.ELEVATOR_HIGH), arm.getPositionCommand(Constants.ARM_RETRACT), elevator.getPositionCommand(Constants.ELEVATOR_LOW));
  SequentialCommandGroup retractArmAndCarryGroup = new SequentialCommandGroup(arm.getPositionCommand(Constants.ARM_GO_BACK), arm.getPositionCommand(Constants.ARM_RETRACT), elevator.getPositionCommand(Constants.ELEVATOR_LOW));
  SequentialCommandGroup placeConeHighGroup = new SequentialCommandGroup(elevator.getPositionCommand(Constants.ELEVATOR_HIGH), arm.getPositionCommand(Constants.ARM_UP));
  SequentialCommandGroup pickUpShelf = new SequentialCommandGroup(elevator.getPositionCommand(Constants.ELEVATOR_MID), arm.getPositionCommand(Constants.ARM_UP));
  SequentialCommandGroup placeConeMiddle = new SequentialCommandGroup(elevator.getPositionCommand(Constants.ELEVATOR_LOW), arm.getPositionCommand(Constants.ARM_UP_MIDDLE));
  SequentialCommandGroup placeCubeMiddle = new SequentialCommandGroup(elevator.getPositionCommand(Constants.ELEVATOR_LOW), arm.getPositionCommand(Constants.ARM_UP_MIDDLE)); 
  SequentialCommandGroup placeCubeHigh = new SequentialCommandGroup(elevator.getPositionCommand(Constants.ELEVATOR_HIGH), arm.getPositionCommand(Constants.ARM_UP));
  String lastcommand = "carry";

  // AUTO
  
  SequentialCommandGroup theEntireAuto = new SequentialCommandGroup(elevator.getPositionCommand(Constants.ELEVATOR_HIGH), arm.getPositionCommand(Constants.ARM_UP), intake.get_intakeCommand(-0.8), arm.getPositionCommand(Constants.ARM_GO_BACK), arm.getPositionCommand(Constants.ARM_RETRACT), elevator.getPositionCommand(Constants.ELEVATOR_LOW));
  


  // AUTO

  /* 
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic(){
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}


  /** This autonomous runs the autonomous com1op  mand selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    elevator.resetEncoderPosition();
    arm.resetEncoderPosition();
    m_robotContainer.m_drivetrainSubsystem.zeroGyroscope();

    arm.setPosition(Constants.ARM_RETRACT, 2);

    theEntireAuto.addCommands(
      m_robotContainer.getAutonomousCommand(1)
    );

    theEntireAuto.schedule();

  }
 
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }


  

  @Override
  public void teleopInit() {    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //SmartDashboard.putNumber("Set Position", 0);
    // System.out.println(group.isScheduled());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Output", elevator.getOutput());
    SmartDashboard.putNumber("Actual Position", elevator.getPosition());

    arm.armsmartdashboard();


    if (controlBoard.getRawButtonPressed(1) && lastcommand.equals("carry")) {
      System.out.println("ran");
      conePickUpGroup.schedule();
      lastcommand = "conepickup";
    } else if (controlBoard.getRawButtonPressed(3) && lastcommand.equals("placeconehigh") || 
               controlBoard.getRawButtonPressed(3) && lastcommand.equals("pickupshelf") || 
               controlBoard.getRawButtonPressed(3) && lastcommand.equals("placecubehigh") ||
               controlBoard.getRawButtonPressed(3) && lastcommand.equals("placecubemiddle") || 
               controlBoard.getRawButtonPressed(3) && lastcommand.equals("placeconemiddle")){
      retractArmAndCarryGroup.schedule();
      lastcommand = "carry";
    } else if (controlBoard.getRawButtonPressed(2)) {
      System.out.println("pressed");
      carryGroup.schedule();
      lastcommand = "carry";
    } else if (controlBoard.getRawButtonPressed(5) && lastcommand.equals("carry")) {
      placeConeHighGroup.schedule();
      lastcommand = "placeconehigh";
    } else if (controlBoard.getRawButtonPressed(7) && lastcommand.equals("carry")) {
      placeConeMiddle.schedule();
      lastcommand = "placeconehigh";
    } else if (controlBoard.getRawButtonPressed(6) && lastcommand.equals("carry")) {
      placeCubeHigh.schedule();
      lastcommand = "placeconehigh";
    } else if (controlBoard.getRawButtonPressed(8) && lastcommand.equals("carry")) {
      placeCubeMiddle.schedule();
      lastcommand = "placeconehigh";
    }

    if (controlBoard.getRawButtonPressed(11)) {
      CommandScheduler.getInstance().cancelAll();
    }

    elevator.elevatorSmartDashboard();

    if (controlIntake.getRightBumperPressed()) {
      intake.driveIntake(0.85);
    } else if (controlIntake.getLeftBumperPressed()) {
      intake.driveIntake(-0.85);
    } else if (controlIntake.getStartButtonPressed()) {
      intake.intakeStop();
    }


  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
