package frc.robot.model;
import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import java.util.List;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import com.pathplanner.lib.commands.FollowPathWithEvents;

public class AutoHandler {
    HashMap<String, Command> events = new HashMap<>();
    PathPlannerTrajectory pathGroup;
    SwerveAutoBuilder builder;

    public AutoHandler(String auto) {
        if (auto == "placeConeHighBalance") {
            events.put("runIntake", new PrintCommand("runIntake"));
            events.put("finishedPiece", new PrintCommand("finishedPiece"));
            pathGroup = PathPlanner.loadPath("placeConeHighBalance", new PathConstraints(3, 2));
        }
    }

    public PathPlannerTrajectory getPathGroup() {
        return pathGroup;
    }

    public HashMap<String, Command> getHashMap() {
        return events;
    }

    public Command getCommandFromBuilder(DrivetrainSubsystem subsystem) {
        return new FollowPathWithEvents(
            subsystem.generatetrajectory(pathGroup, true),
            pathGroup.getMarkers(),
            events
        );
    }

}
