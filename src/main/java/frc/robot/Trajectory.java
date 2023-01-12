package frc.robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.commands.DefaultDriveCommand;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import com.pathplanner.lib.PathPlannerTrajectory;
import frc.robot.Constants;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import java.util.HashMap;

public class Trajectory {
    public Trajectory(String filename){
        String finalfilename = filename + ".path";
        PathPlannerTrajectory path = PathPlanner.loadPath(finalfilename, new PathConstraints(4, 3));
        HashMap<String, Command> eventMap = new HashMap<>();
        PathPlannerState pathstate = (PathPlannerState) path.sample(1.2);
        eventMap.put("marker1", new PrintCommand("passed marker 1"));

    }
}
