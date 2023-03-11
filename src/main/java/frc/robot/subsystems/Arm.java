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
        a_follower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        a_leader = new CANSparkMax(Constants.LEFT_ARM_MOTOR, MotorType.kBrushless);
        a_leader.setIdleMode(CANSparkMax.IdleMode.kCoast);

        a_leaderController = a_leader.getPIDController();
        a_leaderEncoder = a_leader.getEncoder();
        a_leaderEncoder.setPositionConversionFactor(100);
        a_leader.setClosedLoopRampRate(.1);
        a_follower.follow(a_leader, true);
        a_followerEncoder = a_follower.getEncoder();
        // a_followerEncoder.setPositionConversionFactor(100);
        a_leaderController.setP(Constants.a_KP, 1);
        a_leaderController.setI(Constants.a_KI, 1);
        a_leaderController.setD(Constants.a_KD, 1);
        a_leaderController.setFF(Constants.a_KF, 1);

        a_leaderController.setP(Constants.a_KPR, 2);
        a_leaderController.setI(Constants.a_KIR, 2);
        a_leaderController.setD(Constants.a_KDR, 2);
        a_leaderController.setFF(Constants.a_KFR, 2);

        a_leaderController.setP(Constants.a_KPUP, 3);
        a_leaderController.setI(Constants.a_KIUP, 3);
        a_leaderController.setD(Constants.a_KDUP, 3);
        a_leaderController.setFF(Constants.a_KFUP, 3);

        a_leaderController.setP(Constants.a_KPB, 0);
        a_leaderController.setI(Constants.a_KIB, 0);
        a_leaderController.setD(Constants.a_KDB, 0);
        a_leaderController.setFF(Constants.a_KFB, 0);

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
        int slot = 1;
        a_leader.setClosedLoopRampRate(.2);

        if (position == Constants.ARM_RETRACT) {
            slot = 2;
            a_leader.setClosedLoopRampRate(1);
        } else if (position == Constants.ARM_UP || position == Constants.ARM_UP_MIDDLE) {
            slot = 3;
            a_leader.setClosedLoopRampRate(.05);
        } else if (position == Constants.ARM_GO_BACK) {
            slot = 0;
            a_leader.setClosedLoopRampRate(2.5);
        }

        final int finalslot = slot;
        return new FunctionalCommand(
            () -> System.out.println("Driving arm"),
            () -> setPosition(position, finalslot),
            interrupted -> System.out.println("Ended driving arm"),
            () -> Math.abs(getArmPosition() - position) <= 400
        );
        //original error 95
    }

    public void setPosition(double position, int slot) {
        a_leaderController.setReference(position, CANSparkMax.ControlType.kPosition, slot);
    }




    public void armsmartdashboard(){
        SmartDashboard.putNumber("arm lead", a_leaderEncoder.getPosition());
        SmartDashboard.putNumber("arm follower", a_followerEncoder.getPosition());
        SmartDashboard.putNumber("Arm Lead Temp", a_leader.getMotorTemperature());
        SmartDashboard.putNumber("Arm Follower temp", a_follower.getMotorTemperature());
        SmartDashboard.putNumber("Voltage", a_leader.getAppliedOutput());
    }

    public void resetEncoderPosition() {
        a_leaderEncoder.setPosition(0.0);
        a_followerEncoder.setPosition(0.0);
    }


    public void setMotorPosition(double rotations) {
        a_leaderController.setReference(rotations, CANSparkMax.ControlType.kPosition, 1);
    }
}

