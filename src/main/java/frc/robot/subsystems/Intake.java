package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private CANSparkMax motor;
    private double elapsedTime = 0.0;

    public Intake() {
        // Initialize motor
        motor = new CANSparkMax(Constants.INTAKE_ID, MotorType.kBrushless);
    }

    // Intake control
    public void driveIntake(double speed) {
        motor.set(speed);
    }
    public void stopIntake() {
        motor.stopMotor();
    }

    // Create base subsystem command
    public FunctionalCommand getIntakeCommand(double speed) {
        return new FunctionalCommand(
            () -> elapsedTime = 0.0,
            () -> driveIntake(speed),
            interrupted -> stopIntake(),
            () -> elapsedTime > 2.0
        );
    }

    // Update periodic values
    @Override
    public void periodic() {
        elapsedTime += (1.0 / 50.0);
    }
    public double getElapsedTime() {
        return elapsedTime;
    }
}
