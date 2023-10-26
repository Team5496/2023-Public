package frc.robot.subsystems;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSwivel {
    TalonSRX motor;
    SparkMaxPIDController pidcontroller;
    RelativeEncoder encoder; 

    public IntakeSwivel() {
        motor = new TalonSRX(Constants.SwivelCANID);
        //encoder = motor.getEncoder();
        //pidcontroller = motor.getPIDController();

        // P, I, D , F -- F stands for functional (which is another multiplier like P)

        //pidcontroller.setP(Constants.s_P);
        //pidcontroller.setI(Constants.s_I);
        //pidcontroller.setD(Constants.s_D);
        //pidcontroller.setFF(Constants.s_F);
    }

    /* UTILITY METHODS */

    public double getSwivelPosition() {
        return motor.getSelectedSensorPosition();
    }

    public void setSwivelPosition(double position) {
        pidcontroller.setReference(position, ControlType.kPosition);
    }

    public FunctionalCommand getCommand(double position) {
        return new FunctionalCommand(
            () -> System.out.println("Intake swivel command initialized for position: " + position),
            () -> setSwivelPosition(position),
            interrupted -> System.out.println("Intake swivel command ended for position: " + position),
            () -> Math.abs(position - getSwivelPosition()) < 100
        );
    }

    public void intakeswivelarmdashboard() {
        SmartDashboard.putNumber("Intake Swivel Ticks", getSwivelPosition());
    }


}
