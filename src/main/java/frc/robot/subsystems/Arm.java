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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.io.IOException;

public class Arm extends SubsystemBase {
    private CANSparkMax a_leader;
    private SparkMaxPIDController a_leaderController;
    private RelativeEncoder a_leaderEncoder;
    public Logger armlogger;
    public ArrayList<String[]> loggingdata;
    public Boolean isdone = false;
    public double goingto = 0.0;

    public Arm(){
        a_leader = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless);
        a_leader.setIdleMode(CANSparkMax.IdleMode.kCoast);

        a_leaderController = a_leader.getPIDController();
        a_leaderEncoder = a_leader.getEncoder();
        a_leaderEncoder.setPositionConversionFactor(100);
        a_leader.setClosedLoopRampRate(0.5);

        a_leaderController.setP(Constants.a_KP, 1);
        a_leaderController.setI(Constants.a_KI, 1);
        a_leaderController.setD(Constants.a_KD, 1);
        a_leaderController.setFF(Constants.a_KF, 1);

        a_leaderController.setOutputRange(Constants.a_OUTPUT_MIN, Constants.a_OUTPUT_MAX, 1);

        armlogger = new Logger("Arm");
        loggingdata = new ArrayList<>();
        
        try {
            armlogger.initialize();
        } catch (IOException e) {
            System.out.println("IO Exception in Arm subsystem constructor");
        }

        
    }

    @Override
    public void periodic() {
        loggingdata.add(new String[]{
            armlogger.getSystemTime(), 
            String.valueOf(a_leaderEncoder.getPosition()),
            String.valueOf(this.goingto),
            String.valueOf(this.isdone),
            String.valueOf(a_leader.getOutputCurrent())
        });

        try {
            armlogger.log(loggingdata);
            loggingdata.clear();
        } catch (IOException exception) {
            System.out.println("IO Exception in Subsystem Elevator: " + exception);
        }
    }


    public double getArmPosition() {
        return a_leaderEncoder.getPosition();

    }

    public FunctionalCommand getPositionCommand(double position) {
        return new FunctionalCommand(
            () -> {
                goingto = position;
                isdone = false;
            },
            () -> setPosition(position, 1),
            interrupted -> isdone = true,
            () -> Math.abs(getArmPosition() - position) <= 400
        );
    }

    public void setPosition(double position, int slot) {
        a_leaderController.setReference(position, CANSparkMax.ControlType.kPosition, slot);
    }

    public void armsmartdashboard(){
        SmartDashboard.putNumber("Arm", a_leaderEncoder.getPosition());
    }

    public void resetEncoderPosition() {
        a_leaderEncoder.setPosition(0.0);
    }


    
}

