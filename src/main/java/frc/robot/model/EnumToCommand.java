package frc.robot.model;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class EnumToCommand {
    HashMap<RobotStates.RobotStatesEnum, SequentialCommandGroup> corresponding_commands = new HashMap<RobotStates.RobotStatesEnum, SequentialCommandGroup>();
  
    public EnumToCommand(Elevator elevator, Arm arm, Intake intake){    
        corresponding_commands.put( 
            RobotStates.RobotStatesEnum.CARRY,
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    arm.getPositionCommand(Constants.ARM_RETRACT),
                    elevator.getPositionCommand(Constants.ELEVATOR_LOW)
                )
            )
        );

        corresponding_commands.put(
            RobotStates.RobotStatesEnum.PICK_UP_LOW,
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    elevator.getPositionCommand(Constants.ELEVATOR_HIGH), 
                    arm.getPositionCommand(Constants.ARM_DOWN)    
                ),
                elevator.getPositionCommand(Constants.ELEVATOR_PICK_UP)
            )
        );

        corresponding_commands.put(
            RobotStates.RobotStatesEnum.RETRACT_W_CARRY,
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    arm.getPositionCommand(Constants.ARM_GO_BACK), 
                    elevator.getPositionCommand(Constants.ELEVATOR_LOW)
                )
            )
        );

        corresponding_commands.put(
            RobotStates.RobotStatesEnum.PLACE_H,
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    elevator.getPositionCommand(Constants.ELEVATOR_HIGH), 
                    arm.getPositionCommand(Constants.ARM_UP)
                )
            )
        );

        corresponding_commands.put(
            RobotStates.RobotStatesEnum.PLACE_M,
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    elevator.getPositionCommand(Constants.ELEVATOR_LOW), 
                    arm.getPositionCommand(Constants.ARM_UP_MIDDLE)
                )
            )
        );

        corresponding_commands.put(
            RobotStates.RobotStatesEnum.PICK_UP_SHELF,
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    elevator.getPositionCommand(Constants.ELEVATOR_SHELF), 
                    arm.getPositionCommand(Constants.ARM_UP_MIDDLE)
                )
            )
        );

    }

    public SequentialCommandGroup getCommand(RobotStates.RobotStatesEnum state) {
        return corresponding_commands.get(state);
    }
}
