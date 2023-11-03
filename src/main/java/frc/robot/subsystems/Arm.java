package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.io.IOException;

import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private CANSparkMax motor;
    private SparkMaxPIDController motorController;
    private RelativeEncoder motorEncoder;

    public Boolean isDone = false;
    public double goingTo = 0.0;

    public Logger logger;
    public ArrayList<String[]> loggingData;

    public Arm(){
    motor = new CANSparkMax(Constants.ARM_ID, MotorType.kBrushless);
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        motorController = motor.getPIDController();
        motorEncoder = motor.getEncoder();
        motorEncoder.setPositionConversionFactor(100);
        motor.setClosedLoopRampRate(0.5);

        motorController.setP(Constants.a_KP, 1);
        motorController.setI(Constants.a_KI, 1);
        motorController.setD(Constants.a_KD, 1);
        motorController.setFF(Constants.a_KF, 1);

        motorController.setOutputRange(Constants.a_OUTPUT_MIN, Constants.a_OUTPUT_MAX, 1);

        logger = new Logger("Arm");
        loggingData = new ArrayList<>();
        
        try {
            logger.initialize();
        } catch (IOException e) {
            System.out.println("IO Exception in Arm subsystem constructor");
        }

    }

    // Logging
    @Override
    public void periodic() {
        loggingData.add(new String[]{
            logger.getSystemTime(), 
            String.valueOf(motorEncoder.getPosition()),
            String.valueOf(this.goingTo),
            String.valueOf(this.isDone),
            String.valueOf(motor.getOutputCurrent())
        });

        try {
            logger.log(loggingData);
            loggingData.clear();
        } catch (IOException exception) {
            System.out.println("IO Exception in Subsystem Elevator: " + exception);
        }
    }

    public double getPosition() {
        return motorEncoder.getPosition();
    }

    public void setPosition(double position, int slot) {
        motorController.setReference(position, CANSparkMax.ControlType.kPosition, slot);
    }

    public FunctionalCommand getPositionCommand(double position) {
        return new FunctionalCommand(
            () -> {
                goingTo = position;
                isDone = false;
            },
            () -> setPosition(position, 1),
            interrupted -> isDone = true,
            () -> Math.abs(getPosition() - position) <= 400
        );
    }

    public void resetEncoderPosition() {
        motorEncoder.setPosition(0.0);
    }

    public void smartDashboard(){
        SmartDashboard.putNumber("Arm", motorEncoder.getPosition());
    }    
}