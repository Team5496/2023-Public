package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax i_leader;
    private double elapsed_time = 0.0;

    public Intake() {
        i_leader = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushed);
    }

    public FunctionalCommand get_intakeCommand(double speed) {
        return new FunctionalCommand(
            () -> elapsed_time = 0.0,
            () -> driveIntake(speed),
            interrupted -> intakeStop(),
            () -> getElapsedTime() > 1.0
        );
    }

    public double getElapsedTime() {
        return elapsed_time;
    }

    @Override
    public void periodic() {
        elapsed_time += (1.0 / 50.0);
    }

    public void driveIntake(double speed) {
        i_leader.set(speed);
    }

    public void intakeStop() {
        i_leader.stopMotor();
    }

}
