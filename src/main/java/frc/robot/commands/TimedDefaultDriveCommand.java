package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.lang.System;

public class TimedDefaultDriveCommand extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;
    private double speedX, speedY, speedZ;
    private long recordedTimeOnInitMs, elapsedTimeSeconds;

    public TimedDefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               double speedX,
                               double speedY,
                               double speedZ,
                               long elapsedTimeSeconds) {

        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.speedX = speedX;
        this.speedY = speedY;
        this.speedZ = speedZ;
        this.elapsedTimeSeconds = elapsedTimeSeconds;
        this.recordedTimeOnInitMs = System.currentTimeMillis();

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        speedX,
                        speedY,
                        speedZ,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );


    }
    public void reset() {
        m_drivetrainSubsystem.zeroGyroscope(0.0);
    }

    @Override
    public boolean isFinished(){
        return System.currentTimeMillis() >= (recordedTimeOnInitMs + elapsedTimeSeconds * 1000);
    }


    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
