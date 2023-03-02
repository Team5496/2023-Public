package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;

public class Intake {
    private CANSparkMax i_leader, i_follower;
    private Servo leftActuator, rightActuator;
    
    public Intake() {

        i_leader = new CANSparkMax(Constants.LEFT_INTAKE_MOTOR, MotorType.kBrushless);
        i_follower = new CANSparkMax(Constants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless);

        i_follower.follow(i_leader);

        leftActuator = new Servo(Constants.LEFT_ACTUATOR);
        leftActuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        rightActuator = new Servo(Constants.RIGHT_ACTUATOR);
        rightActuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    }

    public void intakeIn(double speed) {
        i_leader.set(speed);
    }
    public void intakeOut(double speed) {
        i_leader.set(speed); //possibly invert, may invert speed when giving the parameter
    }
    public void intakeStop() {
        i_leader.stopMotor();
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
