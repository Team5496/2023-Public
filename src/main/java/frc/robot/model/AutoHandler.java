package frc.robot.model;
import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import java.util.List;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;

public class AutoHandler {
    HashMap<String, Command> events = new HashMap<>();
    List<PathPlannerTrajectory> pathGroup;
    SwerveAutoBuilder builder;

    public AutoHandler(String auto) {
        if (auto == "placeConeHighBalance") {
            events.put("runIntake", new PrintCommand("Intake ran!!"));
            pathGroup = PathPlanner.loadPathGroup("placeConeHighBalance", new PathConstraints(3, 2));
        }
    }

    public HashMap<String, Command> getHashMap() {
        return events;
    }

    public Command getCommandFromBuilder(SwerveAutoBuilder builder) {
        return builder.fullAuto(
            pathGroup
        );
    }

}
