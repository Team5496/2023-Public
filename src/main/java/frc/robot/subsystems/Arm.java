package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.Map;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;


public class Arm {
    private CANSparkMax a_follower, a_leader;
    private SparkMaxPIDController a_leaderPID;
    private RelativeEncoder a_followerEncoder, a_leaderEncoder;
    DigitalInput m_sensor = new DigitalInput(1);

    public boolean getArmSensor() {
        return m_sensor.get();
    }

    public Arm(){
        super();

        a_follower = new CANSparkMax(Constants.RIGHT_ARM_MOTOR, MotorType.kBrushless);
        a_leader = new CANSparkMax(Constants.LEFT_ARM_MOTOR, MotorType.kBrushless);
        
        a_leaderPID = a_follower.getPIDController();
        a_leaderEncoder = a_leader.getEncoder();
        a_leaderEncoder.setPositionConversionFactor(100);
        a_leader.setClosedLoopRampRate(.1);

        a_follower.follow(a_leader, true);
        a_followerEncoder = a_follower.getEncoder();
        a_followerEncoder.setPositionConversionFactor(100);

        a_leaderPID.setP(Constants.a_KP, 1);
        a_leaderPID.setI(Constants.a_KI, 1);
        a_leaderPID.setD(Constants.a_KD, 1);
        a_leaderPID.setFF(Constants.a_KF, 1);
        a_leaderPID.setOutputRange(Constants.a_OUTPUT_MIN, Constants.a_OUTPUT_MAX, 1);
    }

    public SequentialCommandGroup setPositionCommandArm(double... positions) {
        SequentialCommandGroup group = new SequentialCommandGroup(new InstantCommand(() -> setMotorPosition(positions[0])));

        for (int i = 1; i < positions.length; i++) {
            final int j = i;
            group.addCommands(
                new InstantCommand(() -> setMotorPosition(positions[j]))
            );
        }
        
        return group;
    }


    public void setMotorPosition(double rotations) {
        a_leaderPID.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }
}
