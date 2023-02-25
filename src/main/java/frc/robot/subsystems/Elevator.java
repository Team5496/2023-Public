package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.SparkMaxPIDController;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

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
        l_motor.setClosedLoopRampRate(.1);

        f_motor.follow(l_motor, true);
        f_motorEncoder = f_motor.getEncoder();
        f_motorEncoder.setPositionConversionFactor(100);

        l_motorController.setP(Constants.e_KP, 0);
        l_motorController.setI(Constants.e_KI, 0);
        l_motorController.setD(Constants.e_KD, 0);
        l_motorController.setFF(Constants.e_KF, 0);
        l_motorController.setOutputRange(Constants.e_OUTPUT_MIN, Constants.e_OUTPUT_MAX, 0);
    }

    public void setPosition(double position) {
        l_motorController.setReference(position, CANSparkMax.ControlType.kPosition);
    }
    public double getPosition() {
        return l_motorEncoder.getPosition();
    }

    public void resetEncoderPosition() {
        l_motorEncoder.setPosition(0);
        f_motorEncoder.setPosition(0);
    }

    public void driveJoystick(double y) {
        l_motor.set(y);
    }
    
    public void goUp() {
        l_motorController.setReference(150.0, CANSparkMax.ControlType.kPosition);
    }

    public void goDown() {
        l_motorController.setReference(-150.0, CANSparkMax.ControlType.kPosition);
    }

    public void hold() {
        l_motorController.setReference(l_motorEncoder.getPosition(), CANSparkMax.ControlType.kVelocity);
    }

    public void elevatorSmartDashboard() {
        SmartDashboard.putNumber("Lead Position", l_motorEncoder.getPosition());
        SmartDashboard.putNumber("Graph Position", l_motorEncoder.getPosition());
        SmartDashboard.putNumber("Follower Position", f_motorEncoder.getPosition());
        SmartDashboard.putNumber("Follower Velocity", f_motorEncoder.getVelocity());
        SmartDashboard.putNumber("Output Current", l_motor.getOutputCurrent());
    }
}
