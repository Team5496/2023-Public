package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.SparkMaxPIDController;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Runnable;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Elevator {
    
    public CANSparkMax l_motor, f_motor;
    private SparkMaxPIDController l_motorController;
    private RelativeEncoder l_motorEncoder, f_motorEncoder;
    public double kP, kI, kD, kF, kMaxOutput, kMinOutput;

    public Elevator() {
        super();
        l_motor = new CANSparkMax(Constants.LEFT_ELEVATOR_MOTOR, MotorType.kBrushless);
        f_motor = new CANSparkMax(Constants.RIGHT_ELEVATOR_MOTOR, MotorType.kBrushless);

        l_motorController = l_motor.getPIDController();
        l_motorEncoder = l_motor.getEncoder();
        l_motorEncoder.setPositionConversionFactor(100);
        l_motor.setClosedLoopRampRate(2.5);
        l_motorEncoder.setPositionConversionFactor(100);
        l_motor.setClosedLoopRampRate(.1);

        f_motor.follow(l_motor, true);
        f_motorEncoder = f_motor.getEncoder();
       // f_motorEncoder.setPositionConversionFactor(100);

        l_motorController.setP(Constants.e_KP, 0);
        l_motorController.setI(Constants.e_KI, 0);
        l_motorController.setD(Constants.e_KD, 0);
        l_motorController.setFF(Constants.e_KF, 0);
        l_motorController.setOutputRange(Constants.e_OUTPUT_MIN, Constants.e_OUTPUT_MAX, 0);
    }


    public void setPosition(double position) {
        l_motorController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public SequentialCommandGroup setPositionCommand(double... positions) {
        SequentialCommandGroup group = new SequentialCommandGroup(new InstantCommand(() -> setPosition(positions[0])));

        for (int i = 0; i < positions.length; i++) {
            final int j = i;
            group.addCommands(
                new InstantCommand(() -> setPosition(positions[j]))
            );
        }

        return group;
    }

    public double getPosition() {
        return l_motorEncoder.getPosition();
    }


    
    public void setPositionRoutine(double... positions) {
        for (double position : positions) {
            setPosition(position);
        }
    }

    public void resetEncoderPosition() {
        l_motorEncoder.setPosition(0);
        f_motorEncoder.setPosition(0);
    }

    public void driveJoystick(double y) {
        l_motor.set(y);
    }
    
    public void goUp() {
        l_motorController.setReference(50, CANSparkMax.ControlType.kVelocity);
    }
    public void goDown() {
        l_motorController.setReference(-50.0, CANSparkMax.ControlType.kVelocity);
    }
    public void hold() {
        l_motorController.setReference(l_motorEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
    }


    public void elevatorSmartDashboard() {
        SmartDashboard.putNumber("Lead Position", l_motorEncoder.getPosition());
        SmartDashboard.putNumber("Graph Position", l_motorEncoder.getPosition());
        SmartDashboard.putNumber("Follower Position", f_motorEncoder.getPosition());
        SmartDashboard.putNumber("Follower Velocity", f_motorEncoder.getVelocity());
        SmartDashboard.putNumber("Output Current", l_motor.getOutputCurrent());
    }
}
