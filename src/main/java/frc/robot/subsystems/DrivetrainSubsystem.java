// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.util.function.Consumer;
import com.pathplanner.lib.commands.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import java.util.function.DoubleSupplier;
import frc.robot.model.RobotStates.RobotStatesEnum;
import frc.robot.Constants;
import frc.robot.commands.DefaultDriveCommand;

import java.util.function.Supplier;
import frc.robot.model.EnumToCommand;
import java.util.HashMap;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.controller.PIDController;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathConstraints;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import java.lang.ArithmeticException;
import static frc.robot.Constants.*;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.geometry.Pose2d;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import frc.robot.commands.TimedDefaultDriveCommand;
import edu.wpi.first.wpilibj.DigitalInput;

public class DrivetrainSubsystem extends SubsystemBase {
  public static double second_timer = 0.0; // lol
  public static double[] distances = {0.0, 0.0, 0.0, 0.0};
  public static final double MAX_VOLTAGE = 12.0;
  public static Limelight limelight = new Limelight("gloworm");
  public static Pose2d m_pose = new Pose2d(0, 0, new Rotation2d());
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  private final Pigeon2 m_pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);

  public double get_yaw() {
        return m_pigeon.getYaw();
  }

  public static double falconToMeters(double positionCounts, double circumference, double gearRatio) {
        return positionCounts * (circumference / (gearRatio * 2048.0));
  }


  // positions are instantiated as 0
  public SwerveModule m_frontLeftModule;
  public SwerveModule m_frontRightModule;
  public SwerveModule m_backLeftModule;
  public SwerveModule m_backRightModule;
  public SwerveModulePosition[] positions;
  private SwerveDriveOdometry m_odometry; 
     
 
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem() {
    super();
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    m_frontLeftModule = new MkSwerveModuleBuilder()
                .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.FALCON, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR)
                .withSteerMotor(MotorType.FALCON, Constants.FRONT_LEFT_MODULE_STEER_MOTOR)
                .withSteerEncoderPort(Constants.FRONT_LEFT_MODULE_STEER_ENCODER)
                .withSteerOffset(Constants.FRONT_LEFT_MODULE_STEER_OFFSET)
                .build();
  
    m_frontRightModule = new MkSwerveModuleBuilder()
                .withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.FALCON, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR)
                .withSteerMotor(MotorType.FALCON, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR)
                .withSteerEncoderPort(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER)
                .withSteerOffset(Constants.FRONT_RIGHT_MODULE_STEER_OFFSET)
                .build();

    m_backLeftModule = new MkSwerveModuleBuilder()
                .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.FALCON, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR)
                .withSteerMotor(MotorType.FALCON, Constants.BACK_LEFT_MODULE_STEER_MOTOR)
                .withSteerEncoderPort(Constants.BACK_LEFT_MODULE_STEER_ENCODER)
                .withSteerOffset(Constants.BACK_LEFT_MODULE_STEER_OFFSET)
                .build();

        
    m_backRightModule = new MkSwerveModuleBuilder()
                .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.FALCON, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR)
                .withSteerMotor(MotorType.FALCON, Constants.BACK_RIGHT_MODULE_STEER_MOTOR)
                .withSteerEncoderPort(Constants.BACK_RIGHT_MODULE_STEER_ENCODER)
                .withSteerOffset(Constants.BACK_RIGHT_MODULE_STEER_OFFSET)
                .build();

        positions = new SwerveModulePosition[]{m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition()};


        m_odometry = new SwerveDriveOdometry(
                m_kinematics,
                getGyroscopeRotation(),
                positions
        );     
                
        
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope(double gyroangle) {
    m_pigeon.setYaw(gyroangle);
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());

}

  public Supplier<Pose2d> get_pose = () -> m_odometry.getPoseMeters(); // returns pose

  public void drive(ChassisSpeeds chassisSpeeds) { // read name
    m_chassisSpeeds = chassisSpeeds;
  }

  public Consumer<Pose2d> reset_poseConsumer = pose -> { // read name
        m_odometry.resetPosition(getGyroscopeRotation(), positions, pose);
  };

  public void resetPoseToPath(Pose2d pose) { // read name
        m_odometry.resetPosition(
                getGyroscopeRotation(),
                new SwerveModulePosition[]{m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition() },
                pose
        );
  }



  public Consumer<SwerveModuleState[]> consume_states = states -> { // swerve consumer, just feeds into drivebase and then purges states
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond  / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians()); 
  };



  @Override
  public void periodic() {
    m_odometry.update(
        getGyroscopeRotation(),
        new SwerveModulePosition[]{ m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(), m_backLeftModule.getPosition(), m_backRightModule.getPosition() }
    );
        
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);    
    
    consume_states.accept(states);
  }


  public double returnZero() {
        return 0.0;
  }

  public double returnZeroPointOne(){
        return 0.1;
  }

  

  public Command generatetrajectory(PathPlannerTrajectory traj, boolean isFirst){
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                  // Reset odometry for the first path you run during auto
                  if(isFirst){
                      this.resetPoseToPath(traj.getInitialHolonomicPose());
                  }
                }),
                new PPSwerveControllerCommand(
                    traj, 
                    get_pose, // Pose supplier
                    m_kinematics, // kP 0.013
                    new PIDController(1.0, 0, 0.0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                    new PIDController(1.0, 0, 0.0), // Y controller (usually the same values as X controller)
                    new PIDController(1.25, 0, 0.0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                    consume_states, // Module states consumer
                    this // Requires this drive subsystem
                )
            );
        
  }


}