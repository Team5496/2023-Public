package frc.robot.model;
import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import java.util.List;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.model.RobotStates.RobotStatesEnum;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import com.pathplanner.lib.commands.FollowPathWithEvents;

public class AutoHandler {
    HashMap<String, Command> events = new HashMap<>();
    List<PathPlannerTrajectory> pathGroup;
    SwerveAutoBuilder builder;
    Command autocommand;

    public AutoHandler(String auto, EnumToCommand enumtocommand, RobotContainer m_container) {
        if (auto == "pickuponepiecebalance") {
            events.put("runIntake", new SequentialCommandGroup(
                new ParallelCommandGroup(
                    enumtocommand.getCommand(RobotStatesEnum.INTAKEON),
                    enumtocommand.getCommand(RobotStatesEnum.PICK_UP_LOW)
                )
            ));

            events.put("intakeup", new SequentialCommandGroup(
                new ParallelCommandGroup(
                    enumtocommand.getCommand(RobotStatesEnum.INTAKEOFF),
                    enumtocommand.getCommand(RobotStatesEnum.RETRACT_W_CARRY)
                )
            ));

            pathGroup = PathPlanner.loadPathGroup("PickUpOnePieceBalance", 
                new PathConstraints(4, 3),
                new PathConstraints(2, 1.5),
                new PathConstraints(4, 3));

            autocommand = m_container.getBuilder(events).fullAuto(pathGroup);
        }
    }

    public List<PathPlannerTrajectory> getPathGroup() {
        return pathGroup;
    }

    public Command getautocommand() {
        return autocommand;
    }

    public HashMap<String, Command> getHashMap() {
        return events;
    }

}
