package frc.robot.model;
import io.statusmachina.core.api.MachineDefinition;
import io.statusmachina.core.api.Transition;
import io.statusmachina.core.stdimpl.EnumBasedMachineDefinitionBuilderProvider;
import static io.statusmachina.core.api.Transition.*;
import frc.robot.model.RobotStates;
import static frc.robot.model.RobotStates.*;

public class RobotStateMachine {
    public enum Objectives {
        PLACE_CUBE_H
    }
    
    final Transition<RobotStatesEnum, Events> t1 = stp(RobotStates.RobotStatesEnum.START, RobotStates.RobotStatesEnum.PICK_UP_C);
    final Transition<RobotStatesEnum, Events> start = event(RobotStatesEnum.START, RobotStatesEnum.PICK_UP_C, Events.START_FINISHED);
    final Transition<RobotStatesEnum, Events> to_place_cube = event(RobotStatesEnum.PICK_UP_C, RobotStatesEnum.PLACE_CUBE_M, Events.CUBE_PICKED_UP);
    final Transition<RobotStatesEnum, Events> cube_placed = event(RobotStatesEnum.PICK_UP_C, RobotStatesEnum.PLACE_CUBE_M, Events.CUBE_PICKED_UP);
   
    // final Transition<States, Events> t3 = event(States.S3, States.S4, Events.E34);
    // final Transition<States, Events> t4 = event(States.S3, States.S5, Events.E35);


    public RobotStateMachine() {
        super();
    }
    /*
     * Public API
     */
    public void achieveObjective(Objectives objective) {
        this.seekObjective(objective);
    }

    private void seekObjective( Objectives objective) {
        switch (objective) {
            case PLACE_CUBE_H: 
            /*
             * Create and run the state machine for place cube H
             * 
            */
            // EnumBasedMachineDefinitionBuilderProvider()
            //     .getMachineDefinitionBuilder(RobotStatesEnum.class, Events.class)
            //     .name("place_cube_high")
            //     .states(RobotStatesEnum.values())
            //     .initialState(RobotStatesEnum.START)
            //     .idleStates(RobotStatesEnum.START)
            //     .terminalStates(RobotStatesEnum.CUBE_PLACED_H)
            //     .events(Events.values())
            //     .transitions(start,to_place_cube, cube_placed)
            //     .build();
        
        }
    }
}
