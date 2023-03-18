package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class IntakeArm {
    private TalonFX m_motor;
    //private TalonSRX b_motor;

    public IntakeArm() {
        m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
		m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);

		m_motor.configNominalOutputForward(0);
		m_motor.configNominalOutputReverse(0);
		m_motor.configPeakOutputForward(0.7);
		m_motor.configPeakOutputReverse(-0.7);

		m_motor.selectProfileSlot(0, 0);
		m_motor.config_kP(0, Constants.IA_KP);
		m_motor.config_kI(0, Constants.IA_KI);
		m_motor.config_kD(0, Constants.IA_KD);
		m_motor.config_kF(0, Constants.IA_KF);

		m_motor.configMotionCruiseVelocity(1434);
		m_motor.configMotionAcceleration(1687);

        /*
        m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
		m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);

		m_motor.configNominalOutputForward(0);
		m_motor.configNominalOutputReverse(0);
		m_motor.configPeakOutputForward(1);
		m_motor.configPeakOutputReverse(-1);

		m_motor.selectProfileSlot(0, 0);
		m_motor.config_kP(0, Constants.IA_KP_BACKUP);
		m_motor.config_kI(0, Constants.IA_KI_BACKUP);
		m_motor.config_kD(0, Constants.IA_KD_BACKUP);
		m_motor.config_kF(0, Constants.IA_KF_BACKUP);

		m_motor.configMotionCruiseVelocity(2389);
		m_motor.configMotionAcceleration(2811);
        */
    }

    public void setPosition(double position) {
        m_motor.set(TalonFXControlMode.MotionMagic, position);
        //b_motor.set(TalonSRXControlMode.MotionMagic, position);
    }

    public void resetEncoder() {
        m_motor.setSelectedSensorPosition(0);
        //b_motor.setSelectedSensorPosition(0);
    }
}
