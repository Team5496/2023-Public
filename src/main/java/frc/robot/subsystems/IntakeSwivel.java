package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import frc.robot.Constants;


public class IntakeSwivel {
    TalonSRX motor;

    public IntakeSwivel() {
        // Initialize motor and settings
        motor = new TalonSRX(Constants.SWIVEL_ID);

        motor.configFactoryDefault();
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
        motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        motor.setNeutralMode(NeutralMode.Coast);
        
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

        motor.configMotionCruiseVelocity(20000 * .5);
		motor.configMotionAcceleration(1e6 * .5);

        motor.enableCurrentLimit(true);
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
            true,
            30,
            35,
            1
        ));

        motor.configContinuousCurrentLimit(30, 60);
        motor.configPeakCurrentLimit(0);
    }

    // Position control
    public double getSwivelPosition() {
        return motor.getSelectedSensorPosition();
    }
    public void setSwivelPosition(double position) {
        motor.set(TalonSRXControlMode.MotionMagic, position);
    }

    // Create base subsystem command
    public FunctionalCommand getPositionCommand(double position) {
        return new FunctionalCommand(
            () -> System.out.println("Intake swivel command initialized for position: " + position),
            () -> setSwivelPosition(position),
            interrupted -> System.out.println("Intake swivel command ended for position: " + position),
            () -> Math.abs(position - getSwivelPosition()) < 200
        );
    }

    // Set encoder value to zero
    public void resetEncoder() {
        motor.setSelectedSensorPosition(0);
    }

    // Put values onto SmartDashboard for testing
    public void smartDashboard() {
        SmartDashboard.putNumber("Intake Swivel Ticks", getSwivelPosition());
        SmartDashboard.putNumber("Intake Swivel Current", motor.getSupplyCurrent());
    }
}
