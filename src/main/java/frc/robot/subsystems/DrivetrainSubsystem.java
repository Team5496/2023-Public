// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.Consumer;
import com.pathplanner.lib.commands.*;
import frc.robot.Constants;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.wpilibj2.command.Command;
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
  public SwerveModulePosition m_frontleftPosition = new SwerveModulePosition(0, new Rotation2d(0, 0));
  public SwerveModule m_frontRightModule;
  public SwerveModulePosition m_frontRightPosition = new SwerveModulePosition(0, new Rotation2d(0, 0));
  public SwerveModule m_backLeftModule;
  public SwerveModulePosition m_backleftPosition = new SwerveModulePosition(0, new Rotation2d(0, 0));
  public SwerveModule m_backRightModule;
  public SwerveModulePosition m_backRightPosition = new SwerveModulePosition(0, new Rotation2d(0, 0));
  public SwerveModulePosition[] positions = new SwerveModulePosition[]{m_frontleftPosition, m_frontRightPosition, m_backleftPosition, m_backRightPosition};
  
  public SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
        m_kinematics, getGyroscopeRotation(), positions, m_pose);
        
     
 
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem() {
    super();
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4iSwerveModuleHelper.GearRatio.L2,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET
    );
  
    m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );
        
    m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
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

  public Supplier<Pose2d> get_pose = () -> m_odometry.getEstimatedPosition(); // returns pose

  public void drive(ChassisSpeeds chassisSpeeds) { // read name
    m_chassisSpeeds = chassisSpeeds;
  }

  public Consumer<Pose2d> reset_poseConsumer = pose -> { // read name
        m_odometry.resetPosition(getGyroscopeRotation(), positions, pose);
  };

  public void reset_poseMethod(Pose2d pose) { // read name
        m_odometry.resetPosition(getGyroscopeRotation(), positions, pose);
  };



  public Consumer<SwerveModuleState[]> consume_states = states -> { // swerve consumer, just feeds into drivebase and then purges states
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond  / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians()); 
  };



  @Override
  public void periodic() {
    second_timer++;
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);


    positions[0] = new SwerveModulePosition(m_frontLeftModule.getDriveVelocity() / 50 + positions[0].distanceMeters, new Rotation2d(m_frontLeftModule.getSteerAngle()));
    positions[1] = new SwerveModulePosition(m_frontRightModule.getDriveVelocity() / 50 + positions[1].distanceMeters, new Rotation2d(m_frontRightModule.getSteerAngle()));
    positions[2] = new SwerveModulePosition(m_backLeftModule.getDriveVelocity() / 50 + positions[2].distanceMeters, new Rotation2d(m_backLeftModule.getSteerAngle()));
    positions[3] = new SwerveModulePosition(m_backRightModule.getDriveVelocity() / 50 + positions[3].distanceMeters, new Rotation2d(m_backRightModule.getSteerAngle()));
    
    
    m_pose = m_odometry.update(getGyroscopeRotation(), positions); // update odometry
    consume_states.accept(states);

  }

  public Command generatetrajectory(PathPlannerTrajectory traj, boolean isFirst){
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                  // Reset odometry for the first path you run during auto
                  if(isFirst){
                      this.reset_poseMethod(traj.getInitialHolonomicPose());
                  }
                }),
                new PPSwerveControllerCommand(
                    traj, 
                    get_pose, // Pose supplier
                    m_kinematics, // kP 0.013
                    new PIDController(0.01, 0, 0.1), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                    new PIDController(0.01, 0, 0.1), // Y controller (usually the same values as X controller)
                    new PIDController(0.01, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                    consume_states, // Module states consumer
                    this // Requires this drive subsystem
                )
            );
        
  }


}