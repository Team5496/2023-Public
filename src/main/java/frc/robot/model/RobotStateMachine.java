package frc.robot.model;
import io.statusmachina.core.api.MachineDefinition;
import io.statusmachina.core.api.Transition;
import io.statusmachina.core.stdimpl.EnumBasedMachineDefinitionBuilderProvider;
import static io.statusmachina.core.api.Transition.*;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.model.RobotStates;
import static frc.robot.model.RobotStates.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.model.EnumToCommand;


public class RobotStateMachine {    
    RobotStates.RobotStatesEnum curr_state;
    SequentialCommandGroup command_forstate;

    final Transition<RobotStatesEnum, Events> t1 = stp(RobotStates.RobotStatesEnum.START, RobotStates.RobotStatesEnum.PICK_UP_C);
    final Transition<RobotStatesEnum, Events> start = event(RobotStatesEnum.START, RobotStatesEnum.PICK_UP_C, Events.START_FINISHED);
    final Transition<RobotStatesEnum, Events> to_place_cube = event(RobotStatesEnum.PICK_UP_C, RobotStatesEnum.PLACE_CUBE_M, Events.CUBE_PICKED_UP);
    final Transition<RobotStatesEnum, Events> cube_placed = event(RobotStatesEnum.PICK_UP_C, RobotStatesEnum.PLACE_CUBE_M, Events.CUBE_PICKED_UP);
    
    // final Transition<States, Events> t3 = event(States.S3, States.S4, Events.E34);
    // final Transition<States, Events> t4 = event(States.S3, States.S5, Events.E35);

    public MachineDefinition<RobotStates.RobotStatesEnum, Events> state_machine;
    private EnumToCommand enumToCommand;

    public RobotStateMachine(Elevator elevator, Arm arm, Intake intake) {
        super();

        enumToCommand = new EnumToCommand(elevator, arm, intake);
    

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
                state_machine = new EnumBasedMachineDefinitionBuilderProvider().getMachineDefinitionBuilder(RobotStatesEnum.class, Events.class)
                    .name("place_cube_high")
                    .states(RobotStatesEnum.values())
                    .initialState(RobotStatesEnum.START)
                    .idleStates(RobotStatesEnum.CARRY)
                    .terminalStates(RobotStatesEnum.CUBE_PLACED_H)
                    .events(Events.values())
                    .transitions(start,to_place_cube, cube_placed)
                    .build();

                

                
        }
    }
}
