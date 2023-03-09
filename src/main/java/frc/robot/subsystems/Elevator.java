package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Elevator {
    
    public CANSparkMax e_leader, e_follower;
    private SparkMaxPIDController e_leaderController;
    private RelativeEncoder e_leaderEncoder, e_followerEncoder;
    public double kP, kI, kD, kF, kMaxOutput, kMinOutput;

    public Elevator() {
        super();

        e_leader = new CANSparkMax(Constants.LEFT_ELEVATOR_MOTOR, MotorType.kBrushless);
        e_follower = new CANSparkMax(Constants.RIGHT_ELEVATOR_MOTOR, MotorType.kBrushless);

        e_leaderController = e_leader.getPIDController();
        e_leaderEncoder = e_leader.getEncoder();
        e_leaderEncoder.setPositionConversionFactor(100);
        e_leader.setClosedLoopRampRate(.3);

        e_follower.follow(e_leader, true);
        e_followerEncoder = e_follower.getEncoder();

        e_leaderController.setP(Constants.e_KP, 0);
        e_leaderController.setI(Constants.e_KI, 0);
        e_leaderController.setD(Constants.e_KD, 0);
        e_leaderController.setFF(Constants.e_KF, 0);
        e_leaderController.setOutputRange(Constants.e_OUTPUT_MIN, Constants.e_OUTPUT_MAX, 0);
    }

    public void resetEncoderPosition() {
        e_leaderEncoder.setPosition(0);
        e_followerEncoder.setPosition(0);
    }

    public void setPosition(double position) {
        if (position < getPosition()) {
            e_leader.setClosedLoopRampRate(2);
        } else {
            e_leader.setClosedLoopRampRate(.6);
        }

        e_leaderController.setReference(position, CANSparkMax.ControlType.kPosition);
    }
    public double getPosition() {
        return e_leaderEncoder.getPosition();
    }


    public FunctionalCommand getPositionCommand(double position) {
        return new FunctionalCommand(
            () -> System.out.println("Driving elevator"),
            () -> setPosition(position),
            interrupted -> System.out.println("Ended driving elevator"),
            () -> Math.abs(getPosition() - position) <= 100
        );
    }

    public SequentialCommandGroup getPositionCommandSequential(double... positions) {
        SequentialCommandGroup group = new SequentialCommandGroup(new InstantCommand(() -> setPosition(positions[0])));
    
        for (int i = 1; i < positions.length; i++) {
            final int j = i;
            group.addCommands(
                new InstantCommand(() -> setPosition(positions[j]))
            );
        }

        return group;
    }
    public void setPositionRoutine(double... positions) {
        for (double position : positions) {
            setPosition(position);
        }
    }


    //experimental/testing methods
    public double getOutput() {
        return e_leader.getOutputCurrent();
    }
    public void driveJoystick(double y) {
        e_leader.set(y);
    }
    public void goUp() {
        e_leaderController.setReference(50, CANSparkMax.ControlType.kVelocity);
    }
    public void goDown() {
        e_leaderController.setReference(-50.0, CANSparkMax.ControlType.kVelocity);
    }
    public void hold() {
        e_leaderController.setReference(e_leaderEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
    }

    public void elevatorSmartDashboard() {
        SmartDashboard.putNumber("Lead Position", e_leaderEncoder.getPosition());
        SmartDashboard.putNumber("Graph Position", e_leaderEncoder.getPosition());
        SmartDashboard.putNumber("Follower Position", e_followerEncoder.getPosition());
        SmartDashboard.putNumber("Follower Velocity", e_followerEncoder.getVelocity());
        SmartDashboard.putNumber("Output Current", e_leader.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Lead Temp", e_leader.getMotorTemperature());
        SmartDashboard.putNumber("Elevator Follower temp", e_follower.getMotorTemperature());
    }
}
