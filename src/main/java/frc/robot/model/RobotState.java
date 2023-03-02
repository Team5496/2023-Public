package frc.robot.model;



public class RobotState {
    public enum DriveState {
        LEFT,
        RIGHT
    }
    public enum IntakeState {
        OUT,
        IN
    }
    public enum ElevatorState {
        BOTTOM,
        LOW,
        MID,
        HIGH
    }
    public enum ArmState {
        STRAIGHT,
        DOWN,
        RETRACT
    }

    private DriveState driveState;
    private IntakeState intakeState;
    private ElevatorState elevatorState;
    private ArmState armState;

    public RobotState(DriveState driveState, IntakeState intakeState, ElevatorState elevatorState, ArmState armState) {
        super();

        this.driveState = driveState;
        this.intakeState = intakeState;
        this.elevatorState = elevatorState;
        this.armState = armState;
    }
    
    public DriveState getDriveState() {
        return driveState;
    }
    public IntakeState getIntakeState() {
        return intakeState;
    }
    public ElevatorState getElevatorState() {
        return elevatorState;
    }
    public ArmState getArmState() {
        return armState;
    }
}
