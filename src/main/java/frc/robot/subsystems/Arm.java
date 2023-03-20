package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;


public class Arm {
    private CANSparkMax a_leader;
    private SparkMaxPIDController a_leaderController;
    private RelativeEncoder a_leaderEncoder;
    DigitalInput m_sensor = new DigitalInput(1);

    public Arm(){
        super();

        a_leader = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless);
        a_leader.setIdleMode(CANSparkMax.IdleMode.kCoast);

        a_leaderController = a_leader.getPIDController();
        a_leaderEncoder = a_leader.getEncoder();
        a_leaderEncoder.setPositionConversionFactor(100);
        a_leader.setClosedLoopRampRate(.1);

        a_leaderController.setP(Constants.a_KP, 1);
        a_leaderController.setI(Constants.a_KI, 1);
        a_leaderController.setD(Constants.a_KD, 1);
        a_leaderController.setFF(Constants.a_KF, 1);

        a_leaderController.setOutputRange(Constants.a_OUTPUT_MIN, Constants.a_OUTPUT_MAX, 1);
    }


    public double getArmPosition() {
        return a_leaderEncoder.getPosition();

    }

    public boolean getArmSensor() {
        return m_sensor.get();
    }

    public FunctionalCommand getPositionCommand(double position) {
        return new FunctionalCommand(
            () -> System.out.println("Driving arm"),
            () -> setPosition(position, 0),
            interrupted -> System.out.println("Ended driving arm"),
            () -> Math.abs(getArmPosition() - position) <= 400
        );
    }

    public void setPosition(double position, int slot) {
        a_leaderController.setReference(position, CANSparkMax.ControlType.kPosition, slot);
    }




    public void armsmartdashboard(){
        SmartDashboard.putNumber("arm lead", a_leaderEncoder.getPosition());
        SmartDashboard.putNumber("Arm Lead Temp", a_leader.getMotorTemperature());
        SmartDashboard.putNumber("Voltage", a_leader.getAppliedOutput());
    }

    public void resetEncoderPosition() {
        a_leaderEncoder.setPosition(0.0);
    }


    
}

