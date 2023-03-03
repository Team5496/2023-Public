package frc.robot.model;

public class RobotStates {
    public enum RobotStatesEnum {
        START,
        PICK_UP_C,
        PICK_UP_UC,
        PICK_UP_TC,
        PLACE_CUBE_H,
        CUBE_PLACED_H,
        PLACE_CUBE_M,
        PLACE_CONE_H,
        PLACE_CONE_M,
        PLACE_LOW,
        STOP,
        CARRY
    }
    enum Events {
        START_FINISHED,
        CUBE_PICKED_UP
    }

    public RobotStates() {
        
    }
}
