package frc.robot.model;

public class RobotStates {
    public enum RobotStatesEnum {
        PICK_UP_LOW,
        PICK_UP_SHELF,
        PLACE_H,
        PLACE_M,
        PLACE_L,
        CARRY,
        RETRACT_W_CARRY
    }

    enum Events {
        START_FINISHED,
        CUBE_PICKED_UP
    }

    RobotStatesEnum curr_state = RobotStatesEnum.CARRY;

    public RobotStates() {
        curr_state = RobotStatesEnum.CARRY;
    }

    public RobotStatesEnum getState() {
        return curr_state;
    }

    public void setState(RobotStatesEnum statetoset) {
        curr_state = statetoset;
    }


}
