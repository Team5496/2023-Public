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
import frc.robot.model.RobotStates.RobotStatesEnum;
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
import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import frc.robot.model.EnumToCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import frc.robot.model.RobotStates;
import frc.robot.model.EnumToCommand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Arm arm = new Arm();
  private Elevator elevator = new Elevator();
  private Intake intake = new Intake();
  private GenericHID controlBoard = new GenericHID(1);

  // TELEOP
  RobotStates curr_state = new RobotStates();
  EnumToCommand enumToCommand = new EnumToCommand(elevator, arm, intake);

  // AUTO
  SequentialCommandGroup placeConeHighAuto = new SequentialCommandGroup(
    elevator.getPositionCommand(Constants.ELEVATOR_HIGH), 
    arm.getPositionCommand(Constants.ARM_UP), 
    intake.get_intakeCommand(-0.8), 
    arm.getPositionCommand(Constants.ARM_GO_BACK), 
    arm.getPositionCommand(Constants.ARM_RETRACT), 
    elevator.getPositionCommand(Constants.ELEVATOR_LOW)
  );

  SequentialCommandGroup placeCubeLowAuto = new SequentialCommandGroup(
    elevator.getPositionCommand(Constants.ELEVATOR_LOW), 
    intake.get_intakeCommand(0.8)
  );

  /* 
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    PathPlannerServer.startServer(5811);  
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
    //elevator.resetEncoderPosition();
    //arm.resetEncoderPosition();
    m_robotContainer.m_drivetrainSubsystem.zeroGyroscope(0.0);


   // arm.setPosition(Constants.ARM_RETRACT, 2);

    m_robotContainer.getAutonomousCommand(1).schedule();
  }
 
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }


  

  @Override
  public void teleopInit() {    
    m_robotContainer.m_drivetrainSubsystem.zeroGyroscope(0.0);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (placeConeHighAuto != null) {
      placeConeHighAuto.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Output", elevator.getOutput());
    SmartDashboard.putNumber("Actual Position", elevator.getPosition());

    arm.armsmartdashboard();

    if (controlBoard.getRawButtonPressed(1)) {
        switch (curr_state.getState()) {
            case CARRY:
            case RETRACT_W_CARRY:
              enumToCommand.getCommand(RobotStatesEnum.PICK_UP_LOW).schedule();
              curr_state.setState(RobotStatesEnum.PICK_UP_LOW);
              break;
            default:
              System.out.println("Bad call for Button 1");
              break;
        }

    } else if (controlBoard.getRawButtonPressed(3)){
        switch (curr_state.getState()) {
            case PLACE_H:
            case PICK_UP_SHELF:
            case PLACE_M:
              enumToCommand.getCommand(RobotStatesEnum.RETRACT_W_CARRY).schedule();
              curr_state.setState(RobotStatesEnum.RETRACT_W_CARRY);
              break;
            default:
              System.out.println("Bad call for Button 2");
              break;
        }
    } else if (controlBoard.getRawButtonPressed(2)) {
        enumToCommand.getCommand(RobotStatesEnum.CARRY).schedule();
        curr_state.setState(RobotStatesEnum.CARRY);
    } else if (controlBoard.getRawButtonPressed(5)) {
        switch (curr_state.getState()) {
            case CARRY:
            case RETRACT_W_CARRY:
              enumToCommand.getCommand(RobotStatesEnum.PLACE_H).schedule();
              curr_state.setState(RobotStatesEnum.PLACE_H);
              break;
            default:
              System.out.println("Bad call for Button 5");
              break;
        }
    } else if (controlBoard.getRawButtonPressed(7)) {
        switch (curr_state.getState()) {
            case CARRY:
            case RETRACT_W_CARRY:
              enumToCommand.getCommand(RobotStatesEnum.PLACE_M).schedule();
              curr_state.setState(RobotStatesEnum.PLACE_M);
              break;
            default:
              System.out.println("Bad call for Button 7");
              break;
        }
    } else if (controlBoard.getRawButtonPressed(4)) {
        switch(curr_state.getState()) {
            case CARRY:
            case RETRACT_W_CARRY:
              enumToCommand.getCommand(RobotStatesEnum.PICK_UP_SHELF).schedule();
              curr_state.setState(RobotStatesEnum.PICK_UP_SHELF);
              break;
            default:
              System.out.println("Bad call for Button 4");
              break;
        }
    }
    
    if (controlBoard.getRawButtonPressed(6)) {
      intake.driveIntake(0.85);
    } else if (controlBoard.getRawButtonPressed(8)) {
      intake.driveIntake(-0.85);
    } else if (controlBoard.getRawButtonPressed(11)) {
      intake.intakeStop();
    }

    elevator.elevatorSmartDashboard();

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
