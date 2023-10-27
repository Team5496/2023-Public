package frc.robot.subsystems;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

public class IntakeSwivel {
    TalonSRX motor;

    public IntakeSwivel() {
        motor = new TalonSRX(Constants.SwivelCANID);

        motor.configFactoryDefault();
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
        motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        
        motor.setSensorPhase(true);
        motor.configNominalOutputForward(0);
		motor.configNominalOutputReverse(0);
		motor.configPeakOutputForward(0.5);
		motor.configPeakOutputReverse(-0.5);

        motor.selectProfileSlot(0, 0);
		motor.config_kP(0, Constants.s_P);
		motor.config_kI(0, Constants.s_I);
		motor.config_kD(0, Constants.s_D);
		motor.config_kF(0, Constants.s_F);

        motor.configMotionCruiseVelocity(20000);
		motor.configMotionAcceleration(1e6);
    }

    /* UTILITY METHODS */

    public double getSwivelPosition() {
        return motor.getSelectedSensorPosition();
    }

    public void resetEncoder() {
        motor.setSelectedSensorPosition(0);
    }

    public void setSwivelPosition(double position) {
        motor.set(TalonSRXControlMode.MotionMagic, position);
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
