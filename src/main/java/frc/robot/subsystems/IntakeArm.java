package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.ArrayList;
import java.io.IOException;
import java.lang.Boolean;


public class IntakeArm extends SubsystemBase {
    public TalonFX m_motor;
    public Logger intakearmlogger;
    public ArrayList<String[]> loggingdata;
	public double goingto = 0.0;
	public Boolean isdone = false;

	// TODO: When we get the Intake Arm, reconfigure PIDs
 
    public IntakeArm() {
		m_motor = new TalonFX(Constants.INTAKE_ARM_ID);
		
		m_motor.configFactoryDefault();
        m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
		m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
		m_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

		m_motor.configNominalOutputForward(0);
		m_motor.configNominalOutputReverse(0);
		m_motor.configPeakOutputForward(1);
		m_motor.configPeakOutputReverse(-1);

		m_motor.selectProfileSlot(0, 0);
		m_motor.config_kP(0, Constants.IA_KP);
		m_motor.config_kI(0, Constants.IA_KI);
		m_motor.config_kD(0, Constants.IA_KD);
		m_motor.config_kF(0, Constants.IA_KF);
 
		m_motor.configMotionCruiseVelocity(30000);
		m_motor.configMotionAcceleration(3e6);

		intakearmlogger = new Logger("Intake Arm");
        loggingdata = new ArrayList<>();
        
        try {
            intakearmlogger.initialize();
        } catch (IOException e) {
            System.out.println("IO Exception in Intake Arm subsystem constructor");
        }

    }

    public void setPosition(double position) {
		//m_motor.config_kF(0, calculateKF(position));
        m_motor.set(TalonFXControlMode.MotionMagic, position);
    }

	@Override
	public void periodic() {
		loggingdata.add(new String[]{
				intakearmlogger.getSystemTime(), 
				String.valueOf(m_motor.getSelectedSensorPosition()),
				String.valueOf(this.goingto),
				String.valueOf(this.isdone),
				String.valueOf(m_motor.getStatorCurrent()),

		});
	
		try {
			intakearmlogger.log(loggingdata);
			loggingdata.clear();
		} catch (IOException exception) {
			System.out.println("IO Exception in Subsystem Intake Arm: " + exception);
		}
	
		m_motor.config_kF(0, calculateKF(m_motor.getSelectedSensorPosition()));
	}

	public double calculateKF(double position) {
		double kf = Constants.IA_KF * Math.sin(
			((position - (double)Constants.VTICKS)/(Constants.HTICKS - Constants.VTICKS)) * (Math.PI/2));

		if (kf < .01) {kf = .01;}
		return kf;
	}

	public FunctionalCommand getIntakeArmCommand(double pos) {
        return new FunctionalCommand(
            () -> {
				isdone = false;
				goingto = pos;
			},
            () -> setPosition(pos),
            interrupted -> isdone = true,
            () -> Math.abs(m_motor.getSelectedSensorPosition() - pos) < 7000
        );
    }

	public void intakeArmSmartDashboard() {
		SmartDashboard.putNumber("Intake Arm Position", m_motor.getSelectedSensorPosition());
	}

   public void resetEncoder() {
        m_motor.setSelectedSensorPosition(0);
    }
}
