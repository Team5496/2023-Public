package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;

import frc.robot.Constants;


public class Intake {
    private CANSparkMax l_motor, f_motor;
    private Servo leftActuator, rightActuator;
    
    public Intake() {
        l_motor = new CANSparkMax(Constants.LEFT_INTAKE_MOTOR, MotorType.kBrushless);
        f_motor = new CANSparkMax(Constants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless);

        f_motor.follow(l_motor);

        leftActuator = new Servo(Constants.LEFT_ACTUATOR);
        leftActuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        rightActuator = new Servo(Constants.RIGHT_ACTUATOR);
        rightActuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    }

    public void intakeIn(double speed) {
        l_motor.set(speed);
    }
    public void intakeOut(double speed) {
        l_motor.set(speed); //possibly invert, may invert speed when giving the parameter
    }
    public void intakeStop() {
        l_motor.stopMotor();
    }

    public void actuatorOut() {
        leftActuator.set(1.0);
        rightActuator.set(1.0);
    }
    public void actuatorIn() {
        leftActuator.set(0.0);
        rightActuator.set(0.0);
    }
}
