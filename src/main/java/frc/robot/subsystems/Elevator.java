package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator {
    private CANSparkMax motors[] = new CANSparkMax[2];
    private SparkMaxPIDController m_pidcontrollers[] = new SparkMaxPIDController[2];
    private int kP, kI, kD, kF;
    private RelativeEncoder m_encoders[] =  new RelativeEncoder[2];
    private DigitalInput m_sensor = new DigitalInput(0);


    public Elevator(int deviceIDs[]){
        super();
        for (int i = 0; i < 2; i++) {
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

    public boolean getElevatorSensor() {
        return m_sensor.get();
    }

    public void driveUp() {

    }
}
