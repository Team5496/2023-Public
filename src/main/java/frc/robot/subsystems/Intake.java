package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private CANSparkMax motor;
    private double elapsedTime = 0.0;

    public Intake() {
        motor = new CANSparkMax(Constants.INTAKE_ID, MotorType.kBrushless);
    }

    public FunctionalCommand getIntakeCommand(double speed) {
        return new FunctionalCommand(
            () -> elapsedTime = 0.0,
            () -> driveIntake(speed),
            interrupted -> stopIntake(),
            () -> elapsedTime > 2.0
        );
    }

    @Override
    public void periodic() {
        elapsedTime += (1.0 / 50.0);
    }
    public double getElapsedTime() {
        return elapsedTime;
    }

    public void driveIntake(double speed) {
        motor.set(speed);
    }
    public void stopIntake() {
        motor.stopMotor();
    }
}
