package frc.robot.model;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.util.HashMap;
import java.util.List;

import frc.robot.model.RobotStates.RobotStatesEnum;
import frc.robot.subsystems.DrivetrainSubsystem;

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

    public HashMap<String, Command> initializeAutoHashMap(EnumToCommand enumToCommand, boolean is_cone) {
        HashMap<String, Command> events = new HashMap<String, Command>();

        events.put("runintake", new SequentialCommandGroup(
          new ParallelCommandGroup(
              enumToCommand.getCommand(RobotStatesEnum.INTAKE_ON, is_cone),
              enumToCommand.getCommand(RobotStatesEnum.PICK_UP_FLOOR, is_cone)
          )
        ));
    
        events.put("intakeup", new SequentialCommandGroup(
          new ParallelCommandGroup(
              enumToCommand.getCommand(RobotStatesEnum.INTAKE_OFF, is_cone),
              enumToCommand.getCommand(RobotStatesEnum.RETRACT_W_CARRY, is_cone)
          )
        ));


        return events;
    
    }

    public Command getAutoCommand(DrivetrainSubsystem drive, HashMap<String, Command> events) {
        drive.resetPoseToPath(pathGroup.get(0).getInitialHolonomicPose());
        return drive.getBuilder(events).fullAuto(pathGroup);
    }


}
