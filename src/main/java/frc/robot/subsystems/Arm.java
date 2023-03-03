package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;


public class Arm {
    private CANSparkMax a_follower, a_leader;
    private SparkMaxPIDController a_leaderController;
    private RelativeEncoder a_followerEncoder, a_leaderEncoder;
    DigitalInput m_sensor = new DigitalInput(1);

    public Arm(){
        super();

        a_follower = new CANSparkMax(Constants.RIGHT_ARM_MOTOR, MotorType.kBrushless);
        a_leader = new CANSparkMax(Constants.LEFT_ARM_MOTOR, MotorType.kBrushless);

        a_leaderController = a_leader.getPIDController();
        a_leaderEncoder = a_leader.getEncoder();
        a_leaderEncoder.setPositionConversionFactor(100);
        a_leader.setClosedLoopRampRate(1);

        a_follower.follow(a_leader, true);
        a_followerEncoder = a_follower.getEncoder();
        // a_followerEncoder.setPositionConversionFactor(100);

        a_leaderController.setP(Constants.a_KP, 1);
        a_leaderController.setI(Constants.a_KI, 1);
        a_leaderController.setD(Constants.a_KD, 1);
        a_leaderController.setFF(Constants.a_KF, 1);
        a_leaderController.setOutputRange(Constants.a_OUTPUT_MIN, Constants.a_OUTPUT_MAX, 1);

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

    public double getArmPosition() {
        return a_leaderEncoder.getPosition();
    }

    public boolean getArmSensor() {
        return m_sensor.get();
    }

    public FunctionalCommand getPositionCommand(double position) {
        return new FunctionalCommand(
            () -> System.out.println("Driving arm"),
            () -> setPosition(position),
            interrupted -> setPosition(0),
            () -> Math.abs(getArmPosition() - position) <= 50
        );
    }

    public void setPosition(double position) {
        a_leaderController.setReference(position, CANSparkMax.ControlType.kPosition);
    }




    public void armsmartdashboard(){
        SmartDashboard.putNumber("arm lead", a_leaderEncoder.getPosition());
        SmartDashboard.putNumber("arm follower", a_followerEncoder.getPosition());
    }

    public void resetEncoderPosition() {
        a_leaderEncoder.setPosition(0.0);
        a_followerEncoder.setPosition(0.0);
    }


    public void setMotorPosition(double rotations) {
        a_leaderController.setReference(rotations, CANSparkMax.ControlType.kPosition, 1);
    }
}

