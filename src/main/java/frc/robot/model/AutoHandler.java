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
    List<PathPlannerTrajectory> pathGroup;

    public AutoHandler(String auto) {
        if (auto == "pickuponepiecebalance") {
            pathGroup = PathPlanner.loadPathGroup("PickUpOnePieceBalance", 
                new PathConstraints(4, 3),
                new PathConstraints(2, 1.5),
                new PathConstraints(4, 3));
        }
    }

    public List<PathPlannerTrajectory> getPathGroup() {
        return pathGroup;
    }

    public HashMap<String, Command> initializeAutoHashMap(EnumToCommand enumToCommand) {
        HashMap<String, Command> events = new HashMap<String, Command>();

        events.put("runintake", new SequentialCommandGroup(
          new ParallelCommandGroup(
              enumToCommand.getCommand(RobotStatesEnum.INTAKEON),
              enumToCommand.getCommand(RobotStatesEnum.PICK_UP_LOW)
          )
        ));
    
        events.put("intakeup", new SequentialCommandGroup(
          new ParallelCommandGroup(
              enumToCommand.getCommand(RobotStatesEnum.INTAKEOFF),
              enumToCommand.getCommand(RobotStatesEnum.RETRACT_W_CARRY)
          )
        ));


        return events;
    
    }

    public Command getautocommand(DrivetrainSubsystem drive, HashMap<String, Command> events) {
        drive.resetPoseToPath(pathGroup.get(0).getInitialHolonomicPose());
        return drive.getBuilder(events).fullAuto(pathGroup);
    }


}
