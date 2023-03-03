package frc.robot.model;
import io.statusmachina.core.api.MachineDefinition;
import io.statusmachina.core.api.Transition;
import io.statusmachina.core.stdimpl.EnumBasedMachineDefinitionBuilderProvider;
import static io.statusmachina.core.api.Transition.*;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.model.RobotStates;
import static frc.robot.model.RobotStates.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;


public class RobotStateMachine {
    HashMap<RobotStates.RobotStatesEnum, SequentialCommandGroup> corresponding_commands = new HashMap<RobotStates.RobotStatesEnum, SequentialCommandGroup>();
    
    RobotStates.RobotStatesEnum curr_state;
    SequentialCommandGroup command_forstate;

    final Transition<RobotStatesEnum, Events> t1 = stp(RobotStates.RobotStatesEnum.START, RobotStates.RobotStatesEnum.PICK_UP_C);
    final Transition<RobotStatesEnum, Events> start = event(RobotStatesEnum.START, RobotStatesEnum.PICK_UP_C, Events.START_FINISHED);
    final Transition<RobotStatesEnum, Events> to_place_cube = event(RobotStatesEnum.PICK_UP_C, RobotStatesEnum.PLACE_CUBE_M, Events.CUBE_PICKED_UP);
    final Transition<RobotStatesEnum, Events> cube_placed = event(RobotStatesEnum.PICK_UP_C, RobotStatesEnum.PLACE_CUBE_M, Events.CUBE_PICKED_UP);
   
    // final Transition<States, Events> t3 = event(States.S3, States.S4, Events.E34);
    // final Transition<States, Events> t4 = event(States.S3, States.S5, Events.E35);

    private Elevator elevator;
    private Arm arm;
    private Intake intake;

    public RobotStateMachine(Elevator elevator, Arm arm, Intake intake) {
        super();

        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
        
        corresponding_commands.put( 
            RobotStates.RobotStatesEnum.CARRY,
            new SequentialCommandGroup(
                elevator.setPositionCommand(Constants.ELEVATOR_MID),
                arm.getPositionCommand(Constants.ARM_RETRACT),
                intake.get_intakeInCommand(10)
            )
        );

        corresponding_commands.put(
            RobotStates.RobotStatesEnum.PICK_UP_C,
            new SequentialCommandGroup(
                elevator.setPositionCommand(Constants.ELEVATOR_LOW),
                arm.getPositionCommand(Constants.ARM_STRAIGHT),
                intake.get_intakeInCommand(10)
            )
        );

        corresponding_commands.put(
            RobotStates.RobotStatesEnum.PICK_UP_UC,
            new SequentialCommandGroup(
                elevator.setPositionCommand(Constants.ELEVATOR_LOW),
                arm.getPositionCommand(Constants.ARM_STRAIGHT),
                intake.get_intakeInCommand(10)
            )
        );

        corresponding_commands.put(
            RobotStates.RobotStatesEnum.PICK_UP_TC,
            new SequentialCommandGroup(
                elevator.setPositionCommand(Constants.ELEVATOR_LOW),
                arm.getPositionCommand(Constants.ARM_STRAIGHT),
                intake.get_intakeInCommand(10)
            )
        );

        corresponding_commands.put(
            RobotStates.RobotStatesEnum.PLACE_LOW,
            new SequentialCommandGroup(
                elevator.setPositionCommand(Constants.ELEVATOR_LOW),
                arm.getPositionCommand(Constants.ARM_DOWN),
                intake.get_intakeOutCommand(10)
            )
        );

        corresponding_commands.put(
            RobotStates.RobotStatesEnum.PLACE_CONE_M,
            new SequentialCommandGroup(
                elevator.setPositionCommand(Constants.ELEVATOR_MID),
                arm.getPositionCommand(Constants.ARM_DOWN),
                intake.get_intakeOutCommand(10)
            )
        );

        corresponding_commands.put(
            RobotStates.RobotStatesEnum.PLACE_CONE_H,
            new SequentialCommandGroup(
                elevator.setPositionCommand(Constants.ELEVATOR_HIGH),
                arm.getPositionCommand(Constants.ARM_DOWN),
                intake.get_intakeOutCommand(10)
            )
        );

        corresponding_commands.put(
            RobotStates.RobotStatesEnum.PLACE_CUBE_M,
            new SequentialCommandGroup(
                elevator.setPositionCommand(Constants.ELEVATOR_MID),
                arm.getPositionCommand(Constants.ARM_STRAIGHT),
                intake.get_intakeOutCommand(10)
            )
        );

        corresponding_commands.put(
            RobotStates.RobotStatesEnum.PLACE_CUBE_H,
            new SequentialCommandGroup(
                elevator.setPositionCommand(Constants.ELEVATOR_HIGH),
                arm.getPositionCommand(Constants.ARM_STRAIGHT),
                intake.get_intakeOutCommand(10)
            )
        );

    }
    /*
     * Public API
     */
    public void achieveObjective(RobotStatesEnum objective) {
        this.seekObjective(objective);
    }

    private void seekObjective(RobotStatesEnum objective) {
        switch (objective) {
            case PLACE_CUBE_H: 
            new EnumBasedMachineDefinitionBuilderProvider()
                .getMachineDefinitionBuilder(RobotStatesEnum.class, Events.class)
                .name("place_cube_high")
                .states(RobotStatesEnum.values())
                .initialState(RobotStatesEnum.START)
                .idleStates(RobotStatesEnum.START)
                .terminalStates(RobotStatesEnum.CUBE_PLACED_H)
                .events(Events.values())
                .transitions(start,to_place_cube, cube_placed)
                .build();
        
        }
    }
}
