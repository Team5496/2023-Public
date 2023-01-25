package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight
{
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    
    NetworkTable limelightTable;
    NetworkTableEntry tx, ty, ta;
    AprilTagFieldLayout aprilTagFieldLayout;

   
    

    public Limelight(String name) {
        camera = new PhotonCamera(name);
        result = camera.getLatestResult();
        limelightTable = NetworkTableInstance.getDefault().getTable("gloworm");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout("C:/Users/mentor/Documents/swervedrivedrop/swerve-template-xbox-Imported/src/main/java/frc/robot/AprilTagFieldLayout.json");
        } catch (IOException e) {
            System.out.println("File not found");
            e.printStackTrace();
        }
        
    }

    //for testing input from camera and output on computer
    public void printID() {
        if(result.hasTargets())
        {        
            System.out.println(result.getBestTarget().getFiducialId());
        }
    }

    //gets the transform that maps the camera space to the tag space
    public Transform3d getCameraToTarget() {
        return result.getBestTarget().getBestCameraToTarget();
    }

    public double getDistanceToTarget() {
        if (result.hasTargets()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                1.18,
                0.36,
                0.0,
                Units.degreesToRadians(result.getBestTarget().getPitch()));
        }
        return 0.0;
    }


    //not working
    public void changePipeline(int index) {
       // limelightTable.getEntry("pipeline").setNumber(index);
        camera.setPipelineIndex(index);
    }

    //values that need to be periodically read/updated
    public void readPeriodically() {
        this.result = camera.getLatestResult();
        tx.getDouble(0.0);
        ty.getDouble(0.0);
        ta.getDouble(0.0);
        //System.out.println("Pipeline" + camera.getPipelineIndex());
    }
}
