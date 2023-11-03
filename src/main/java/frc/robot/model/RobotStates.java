package frc.robot.model;

public class RobotStates {
    public enum RobotStatesEnum {
        PICK_UP_FLOOR,
        PICK_UP_RAMP,
        PLACE_H,
        PLACE_M,
        //PLACE_L,
        //PLACE_CUBE_AUTO,
        //PLACE_CONE_AUTO,
        PLACE_CUBE_MID_AUTO,
        INTAKE_ON,
        INTAKE_OFF,
        CARRY,
        RETRACT_W_CARRY
    }

    enum Events {
        START_FINISHED,
        CUBE_PICKED_UP
    }

    RobotStatesEnum currState = RobotStatesEnum.CARRY;

    public RobotStates() {
        currState = RobotStatesEnum.CARRY;
    }

    public RobotStatesEnum getState() {
        return currState;
    }

    public void setState(RobotStatesEnum stateToSet) {
        currState = stateToSet;
    }


}
