// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
  private final Joystick m_codriver = new Joystick(1);
  private Arm arm = new Arm();
  private Elevator elevator = new Elevator();
  private Intake intake = new Intake();
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private EnumToCommand teleopcommandhandler = new EnumToCommand(elevator, arm, intake);
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color colorYellow = new Color(0.322265625, 0.567138671875, 0.111083984375);
  private final Color colorPurple = new Color(0.174560546875, 0.30712890625, 0.5185546875);
  private String gamepiece = "";
  private DigitalInput magnet = new DigitalInput(0);

  private Boolean foundapriltag = false;
  private double recordedapriltagdistanceforpath = 0.0;
  private int recordedapriltagID = 0;
  private Limelight limelight = new Limelight("gloworm");
  private final XboxController m_normaldriver = new XboxController(2);
  SequentialCommandGroup conePickUpGroup = new SequentialCommandGroup(elevator.getPositionCommand(Constants.ELEVATOR_HIGH), arm.getPositionCommand(Constants.ARM_DOWN), elevator.getPositionCommand(Constants.ELEVATOR_PICK_UP));
  SequentialCommandGroup carryGroup = new SequentialCommandGroup(elevator.getPositionCommand(Constants.ELEVATOR_HIGH), arm.getPositionCommand(Constants.ARM_RETRACT), elevator.getPositionCommand(Constants.ELEVATOR_LOW));
  SequentialCommandGroup armgoback = new SequentialCommandGroup(arm.getPositionCommand(Constants.ARM_GO_BACK));
  SequentialCommandGroup placeConeHighGroup = new SequentialCommandGroup(elevator.getPositionCommand(Constants.ELEVATOR_HIGH), arm.getPositionCommand(Constants.ARM_STRAIGHT));
  SequentialCommandGroup testarmgoingup = new SequentialCommandGroup(elevator.getPositionCommand(Constants.ELEVATOR_HIGH), arm.getPositionCommand(Constants.ARM_UP));

  //SequentialCommandGroup start = new SequentialCommandGroup(elevator.setPositionCommand(Constants.ELEVATOR_MID), arm.getPositionCommand(Constants.ARM_RETRACT));

  /* 
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();    
    m_colorMatcher.addColorMatch(colorPurple);
    m_colorMatcher.addColorMatch(colorYellow);
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
    Color detectedcolor =  m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedcolor);
  
    if (match.color == colorYellow) {
      gamepiece = "cone";
    } else {
      gamepiece = "cube";
    }

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
      m_robotContainer.m_drivetrainSubsystem.zeroGyroscope();
      for (int i = 0; i < 2; i++) {
        m_autonomousCommands[i] = m_robotContainer.getAutonomousCommand("right", i);
      }
 
      SequentialCommandGroup two_part_auto = new SequentialCommandGroup(m_autonomousCommands[0], m_autonomousCommands[1]);
      two_part_auto.schedule();
  }
 
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    limelight.readPeriodically();

    if (limelight.getCameraToTarget() != null) {
        recordedapriltagdistanceforpath = limelight.getDistanceToTarget();
        recordedapriltagID = limelight.getID();
    }

   if (true) {
     foundapriltag = false;
      System.out.println("finished");

      if (gamepiece == "cube") {Command command = m_robotContainer.getSimpleCommand("right"); command.schedule();}
      else {
        Command command = m_robotContainer.getSimpleCommand("left"); command.schedule();
      }
    }
  }


  

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.m_drivetrainSubsystem.zeroGyroscope();
    SmartDashboard.putNumber("kP", Constants.a_KP);
    SmartDashboard.putNumber("kI", Constants.a_KI);
    SmartDashboard.putNumber("kF", Constants.a_KF);

    elevator.resetEncoderPosition();
    arm.resetEncoderPosition();
    arm.setPosition(Constants.ARM_RETRACT, 2);
    SmartDashboard.putNumber("Set Position", 0);
    // System.out.println(group.isScheduled());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Output", elevator.getOutput());
    SmartDashboard.putNumber("Actual Position", elevator.getPosition());

    arm.armsmartdashboard();

    if (m_normaldriver.getAButton()) {
      conePickUpGroup.schedule();
    } else if (m_normaldriver.getBButton()) {
      carryGroup.schedule();
    } else if (m_normaldriver.getXButtonPressed()) {
      armgoback.schedule();
    } else if (m_normaldriver.getYButtonPressed()) {
      testarmgoingup.schedule();
    }

    if (m_normaldriver.getBackButtonPressed()) {
      elevator.setPosition(elevator.getPosition() - 100);
    }

    if (m_normaldriver.getStartButtonPressed()) {
      elevator.setPosition(elevator.getPosition() + 100);
    }
    

    if(m_normaldriver.getLeftBumperPressed()) {
      intake.intakeIn(0);
    } else if(m_normaldriver.getRightBumperPressed()) {
      intake.intakeIn(0.8);
    } else if  (m_normaldriver.getRightTriggerAxis() > 0.1) {
      intake.intakeOut(-0.8);
    }

    elevator.elevatorSmartDashboard();
    /* 
    if (m_normaldriver.getYButtonPressed() && ran == false){
      elevatorroutine.schedule();
      ran = true;
    }
     
    if (magnet.get()) {elevator.goUp();}
    else {
      elevator.goDown();
    }
    */

    /* 
    limelight.readPeriodically();
    if (limelight.getCameraToTarget() != null) {
      switch(recordedapriltagID)
      {
        case 6:
        
        case 7:
        case 8:
        case 3:
        case 2:
        case 1:

      }
    
    }

    */


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
