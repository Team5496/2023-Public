package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Arm {
    private CANSparkMax motors[] = new CANSparkMax[3];
    private SparkMaxPIDController m_pidcontrollers[] = new SparkMaxPIDController[3];
    private int kP, kI, kD, kF;
    private RelativeEncoder m_encoders[] =  new RelativeEncoder[3];

    public Arm(int deviceIDs[]){
        for (int i = 0; i < 3; i++) {
            motors[i] = new CANSparkMax(deviceIDs[i], MotorType.kBrushless);
            m_pidcontrollers[i] = motors[i].getPIDController();
            m_encoders[i] = motors[i].getEncoder();

            m_pidcontrollers[i].setP(kP);
            m_pidcontrollers[i].setI(kI);
            m_pidcontrollers[i].setD(kD);
            m_pidcontrollers[i].setFF(kF);
            m_pidcontrollers[i].setOutputRange(-1, 1);
        }
    }

    public void setMotorPosition(int deviceIndex, int rotations) {
        m_pidcontrollers[deviceIndex].setReference(rotations, CANSparkMax.ControlType.kPosition);
    }
}
