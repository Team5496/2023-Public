package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight
{
    private PhotonCamera camera;
    public PhotonPipelineResult result;
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    NetworkTable limelightTable;
    double tx, ty, tv;
    AprilTagFieldLayout aprilTagFieldLayout;

    public Limelight(String name) {
        camera = new PhotonCamera(name);
        result = camera.getLatestResult();
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");


        //tx = limelightTable.getEntry("tx");
        //ty = limelightTable.getEntry("ty");
        //ta = limelightTable.getEntry("ta");

        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "AprilTagFieldLayout.json");
        } catch (IOException e) {
            System.out.println("File not found");
            e.printStackTrace();
        }
    }

    //for testing input from camera and output on computer
    public void getTargetInformation() {
        if(result.hasTargets())
        {        
            SmartDashboard.putNumber("id", result.getBestTarget().getFiducialId());
            SmartDashboard.putNumber("yaw", result.getBestTarget().getYaw());
            SmartDashboard.putNumber("pitch", result.getBestTarget().getPitch());
            SmartDashboard.putNumber("skew", result.getBestTarget().getSkew());
            SmartDashboard.putNumber("area", result.getBestTarget().getArea());

            SmartDashboard.putNumber("tv", tv);
            SmartDashboard.putNumber("tx", tx);
        }
    }

    //gets the transform that maps the camera space to the tag space
    public Transform3d getCameraToTarget() {
        if(result.hasTargets()) {
            Transform3d transform = result.getBestTarget().getBestCameraToTarget();
            Translation3d translation = transform.getTranslation();
            translation.getX();
        }
        return result.hasTargets() ? result.getBestTarget().getBestCameraToTarget()
            : null; 
    }
    public boolean isCentered() {
        if(result.hasTargets()) {
            if(result.getBestTarget().getYaw() == 0.0 && result.getBestTarget().getSkew() == 0.0) {
                return true;
            }
        }
        return false;
    }
    //where the first object is yaw, and the second is skew
    public double[] getDistanceToCenter() {
        double[] returnValue = new double[2];
        if(result.hasTargets()) {
            returnValue[0] = result.getBestTarget().getSkew();
            returnValue[1] = result.getBestTarget().getYaw();
            System.out.println(returnValue[0] + " " + returnValue[1]);
        }
        else {
            System.out.println("No target");
        }
        return returnValue;
    }   

    //gets the distance from the camera to the target
    public double getDistanceToTarget() {
        if (result.hasTargets()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                1.22,
                0.36,
                0.0,
                Units.degreesToRadians(result.getBestTarget().getPitch()));
        }
        return 0.0;
    }
    public int getID() {
        return result.hasTargets() ? result.getBestTarget().getFiducialId()
            : 0;
    }


    //not working
    public void changePipeline(int index) {
       // limelightTable.getEntry("pipeline").setNumber(index);
        camera.setPipelineIndex(index);
    }

    //values that need to be periodically read/updated
    public void readPeriodically() {
        this.result = camera.getLatestResult();
        tx = limelightTable.getEntry("tx").getDouble(0.0);
        ty = limelightTable.getEntry("ty").getDouble(0.0);
        tv = limelightTable.getEntry("tv").getDouble(0.0);

        for(String str : limelightTable.getKeys()) {
            System.out.println(str);
        }
        //ty.getDouble(0.0);
        //ta.getDouble(0.0);
        //System.out.println("Pipeline" + camera.getPipelineIndex());
    }
}