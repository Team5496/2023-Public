package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class Intake {
    private CANSparkMax i_leader;    

    public Intake() {
        i_leader = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushed);
    }

    public FunctionalCommand get_intakeInCommand(double speed) {
        return new InstantCommand(() -> intakeIn(speed));
    }

    public FunctionalCommand get_intakeOutCommand(double speed) {
        return new InstantCommand(() -> intakeOut(speed));
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

}
