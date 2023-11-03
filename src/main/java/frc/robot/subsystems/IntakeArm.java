package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import java.lang.Boolean;
import java.util.ArrayList;
import java.io.IOException;

import frc.robot.Constants;

public class IntakeArm extends SubsystemBase {
    public TalonFX motor;

	public double goingTo = 0.0;
	public Boolean isDone = false;

    public Logger logger;
    public ArrayList<String[]> loggingData;

	// TODO: When we get the Intake Arm, reconfigure PIDs
 
    public IntakeArm() {
		motor = new TalonFX(Constants.INTAKE_ARM_ID);
		
		motor.configFactoryDefault();
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
		motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

		motor.configNominalOutputForward(0);
		motor.configNominalOutputReverse(0);
		motor.configPeakOutputForward(1);
		motor.configPeakOutputReverse(-1);

		motor.selectProfileSlot(0, 0);
		motor.config_kP(0, Constants.ia_KP);
		motor.config_kI(0, Constants.ia_KI);
		motor.config_kD(0, Constants.ia_KD);
		motor.config_kF(0, Constants.ia_KF);
 
		motor.configMotionCruiseVelocity(20000);
		motor.configMotionAcceleration(1e6);

		logger = new Logger("Intake Arm");
        loggingData = new ArrayList<>();
        
        try {
            logger.initialize();
        } catch (IOException e) {
            System.out.println("IO Exception in Intake Arm subsystem constructor");
        }

    }

	// Logging
	@Override
	public void periodic() {
		loggingData.add(new String[]{
				logger.getSystemTime(), 
				String.valueOf(motor.getSelectedSensorPosition()),
				String.valueOf(this.goingTo),
				String.valueOf(this.isDone),
				String.valueOf(motor.getStatorCurrent()),

		});
	
		try {
			logger.log(loggingData);
			loggingData.clear();
		} catch (IOException exception) {
			System.out.println("IO Exception in Subsystem Intake Arm: " + exception);
		}
	
		motor.config_kF(0, calculateKF(motor.getSelectedSensorPosition()));
	}

	public double calculateKF(double position) {
		double kf = Constants.ia_KF * Math.sin(
			((position - (double)Constants.VTICKS)/(Constants.HTICKS - Constants.VTICKS)) * (Math.PI/2));

		if (kf < .01) {kf = .01;}
		return kf;
	}

    public void setPosition(double position) {
		//m_motor.config_kF(0, calculateKF(position));
        motor.set(TalonFXControlMode.MotionMagic, position);
    }

	public FunctionalCommand getIntakeArmCommand(double position) {
        return new FunctionalCommand(
            () -> {
				isDone = false;
				goingTo = position;
			},
            () -> setPosition(position),
            interrupted -> isDone = true,
            () -> Math.abs(motor.getSelectedSensorPosition() - position) < 7000
        );
    }

   public void resetEncoder() {
        motor.setSelectedSensorPosition(0);
    }

	public void smartDashboard() {
		SmartDashboard.putNumber("Intake Arm Position", motor.getSelectedSensorPosition());
	}	
}
