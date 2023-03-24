// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.model.AutoHandler;
import frc.robot.model.EnumToCommand;
import frc.robot.model.RobotStates;
import frc.robot.model.RobotStates.RobotStatesEnum;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private EnumToCommand enumToCommand;

  // TELEOP

  private GenericHID controlBoard = new GenericHID(1);
  RobotStates curr_state = new RobotStates();
  public XboxController intakecontroller = new XboxController(2);

  // AUTO

  private AutoHandler autoHandler = new AutoHandler("placeConeHighBalance");

  /*

  SequentialCommandGroup placeConeHighAuto = new SequentialCommandGroup(
    m_robotContainer.m_elevator.getPositionCommand(Constants.ELEVATOR_HIGH), 
    m_robotContainer.m_arm.getPositionCommand(Constants.ARM_UP), 
    m_robotContainer.m_intake.get_intakeCommand(-0.8), 
    m_robotContainer.m_arm.getPositionCommand(Constants.ARM_GO_BACK), 
    m_robotContainer.m_arm.getPositionCommand(Constants.ARM_RETRACT), 
    m_robotContainer.m_elevator.getPositionCommand(Constants.ELEVATOR_LOW)
  );

  SequentialCommandGroup placeCubeLowAuto = new SequentialCommandGroup(
    m_robotContainer.m_elevator.getPositionCommand(Constants.ELEVATOR_LOW), 
    m_robotContainer.m_intake.get_intakeCommand(0.8)
  );

  */

  /* 
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    enumToCommand = new EnumToCommand(m_robotContainer.m_elevator, m_robotContainer.m_arm, m_robotContainer.m_intake, m_robotContainer.m_intakearm);


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
    m_robotContainer.m_elevator.resetEncoderPosition();
    m_robotContainer.m_arm.resetEncoderPosition();
    m_robotContainer.m_intakearm.resetEncoder();
    m_robotContainer.m_drivetrainSubsystem.zeroGyroscope(0.0);
    m_robotContainer.m_intakearm.m_motor.configMotionCruiseVelocity(15000);

    SequentialCommandGroup scg = (SequentialCommandGroup) enumToCommand.getCommand(RobotStatesEnum.PLACE_CUBE_AUTO);
    scg.schedule();

   // m_robotContainer.getAutonomousCommand(1).schedule();

  // m_robotContainer.m_drivetrainSubsystem.reset_poseMethod(autoHandler.getPathGroup().getInitialHolonomicPose());

   //autoHandler.getCommandFromBuilder(m_robotContainer.m_drivetrainSubsystem).schedule();
  }
 
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_robotContainer.m_intakearm.intakeArmSmartDashboard();
    m_robotContainer.m_arm.armsmartdashboard();
    m_robotContainer.m_elevator.elevatorSmartDashboard();

  }


  

  @Override
  public void teleopInit() {
    m_robotContainer.m_intakearm.m_motor.configMotionCruiseVelocity(30000);


    m_robotContainer.m_elevator.setPosition(500);
    m_robotContainer.m_arm.setPosition(0, 1);
    m_robotContainer.m_intakearm.setPosition(0);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.m_intakearm.intakeArmSmartDashboard();
    m_robotContainer.m_arm.armsmartdashboard();
    m_robotContainer.m_elevator.elevatorSmartDashboard();

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
    } else if (controlBoard.getRawButtonPressed(2)) {
        switch (curr_state.getState()) {
          case PICK_UP_CHUTE:
          case PICK_UP_CHUTE_CONE:
          case PICK_UP_LOW:
            enumToCommand.getCommand(RobotStatesEnum.RETRACT_W_CARRY).schedule();
            curr_state.setState(RobotStatesEnum.CARRY);
            break;
          default:
            enumToCommand.getCommand(RobotStatesEnum.CARRY).schedule();
            curr_state.setState(RobotStatesEnum.CARRY);
            break;
        }
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
              enumToCommand.getCommand(RobotStatesEnum.PICK_UP_CHUTE).schedule();
              curr_state.setState(RobotStatesEnum.PICK_UP_CHUTE);
              break;
            default:
              System.out.println("Bad call for Button 4");
              break;
        }
    } else if (controlBoard.getRawButtonPressed(3)) {
        switch (curr_state.getState()) {
            case CARRY:
            case RETRACT_W_CARRY:
              enumToCommand.getCommand(RobotStatesEnum.PICK_UP_CHUTE_CONE).schedule();
              curr_state.setState(RobotStatesEnum.PICK_UP_CHUTE_CONE);
              break;
            default:
              System.out.println("Bad call for Button 3");
              break;
        }
    }
    
    if (controlBoard.getRawButtonPressed(6)) {
      m_robotContainer.m_intake.driveIntake(0.85);
    } else if (controlBoard.getRawButtonPressed(8)) {
      m_robotContainer.m_intake.driveIntake(-0.85);
    } else if (controlBoard.getRawButtonPressed(11)) {
      m_robotContainer.m_intake.intakeStop();
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
