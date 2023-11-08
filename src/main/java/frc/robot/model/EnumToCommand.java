package frc.robot.model;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.HashMap;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeSwivel;

public class EnumToCommand {
    HashMap<RobotStates.RobotStatesEnum, Command> correspondingCommandsCube = new HashMap<RobotStates.RobotStatesEnum, Command>();
    HashMap<RobotStates.RobotStatesEnum, Command> correspondingCommandsCone = new HashMap<RobotStates.RobotStatesEnum, Command>();

    public EnumToCommand(Elevator elevator, Arm arm, Intake intake, IntakeArm intakeArm, IntakeSwivel intakeSwivel){    
    // CARRY
        // CONE
        correspondingCommandsCone.put( 
            RobotStates.RobotStatesEnum.CARRY,
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    elevator.getPositionCommand(Constants.ELEVATOR_CARRY),
                    arm.getPositionCommand(Constants.ARM_CARRY),
                    intakeArm.getPositionCommand(Constants.INARM_CARRY)
                ),
                new ParallelCommandGroup(
                    intakeSwivel.getPositionCommand(Constants.SWIVEL_CARRY),
                    intake.getIntakeCommand(Constants.INTAKE_BACKWARD_HOLD)
                )
            )
        );

        // CUBE
        correspondingCommandsCube.put( 
            RobotStates.RobotStatesEnum.CARRY,
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    elevator.getPositionCommand(Constants.ELEVATOR_CARRY),
                    arm.getPositionCommand(Constants.ARM_CARRY),
                    intakeArm.getPositionCommand(Constants.INARM_CARRY)
                ),
                new ParallelCommandGroup(
                    intakeSwivel.getPositionCommand(Constants.SWIVEL_CARRY),
                    intake.getIntakeCommand(Constants.INTAKE_FORWARD_HOLD)
                )
            )
        );


    // PICK UP LOW
        // CONE
        correspondingCommandsCone.put(
            RobotStates.RobotStatesEnum.PICK_UP_FLOOR,
            new ParallelCommandGroup(
                elevator.getPositionCommand(0),
                arm.getPositionCommand(0),
                intakeArm.getPositionCommand(Constants.INARM_FLOOR_CONE),
                intakeSwivel.getPositionCommand(Constants.SWIVEL_FLOOR_CONE) 
            )
        );

        // CUBE --> not tuned for new intake
        correspondingCommandsCube.put(
            RobotStates.RobotStatesEnum.PICK_UP_FLOOR,
            new ParallelCommandGroup(
                elevator.getPositionCommand(0),
                arm.getPositionCommand(0),
                intakeArm.getPositionCommand(Constants.INARM_FLOOR_CUBE), // NO VALUE
                intakeSwivel.getPositionCommand(Constants.SWIVEL_FLOOR_CUBE) // NEED TO CHECK
            )
        );

    // INTAKE
        // INTAKE ON
        correspondingCommandsCone.put(
            RobotStates.RobotStatesEnum.INTAKE_ON,
            intake.getIntakeCommand(Constants.INTAKE_BACKWARD)
        );

        correspondingCommandsCube.put(
            RobotStates.RobotStatesEnum.INTAKE_ON,
            intake.getIntakeCommand(Constants.INTAKE_FORWARD)
        );

        // INTAKE OFF
        correspondingCommandsCone.put(
            RobotStates.RobotStatesEnum.INTAKE_OFF,
            intake.getIntakeCommand(0.0)
        );

        correspondingCommandsCube.put(
            RobotStates.RobotStatesEnum.INTAKE_OFF,
            intake.getIntakeCommand(0.0)
        );


    // RETRACT W CARRY
        // CONE
        correspondingCommandsCone.put(
            RobotStates.RobotStatesEnum.RETRACT_W_CARRY,
            new ParallelCommandGroup(
                elevator.getPositionCommand(0),
                arm.getPositionCommand(0),
                intakeArm.getPositionCommand(0),
                intakeSwivel.getPositionCommand(0)
            )
        );

        // CUBE
        correspondingCommandsCube.put(
            RobotStates.RobotStatesEnum.RETRACT_W_CARRY,
            new ParallelCommandGroup(
                elevator.getPositionCommand(0),
                arm.getPositionCommand(0),
                intakeArm.getPositionCommand(0),
                intakeSwivel.getPositionCommand(0)
            )
        );

    // PLACE HIGH
        // CONE
        correspondingCommandsCone.put(
            RobotStates.RobotStatesEnum.PLACE_H,
            new SequentialCommandGroup(
                elevator.getPositionCommand(Constants.ELEVATOR_HIGH_CONE),
                new ParallelCommandGroup(
                    intake.getIntakeCommand(Constants.INTAKE_BACKWARD_HOLD),
                    intakeSwivel.getPositionCommand(Constants.SWIVEL_HIGH_CONE)
                ),
                new ParallelCommandGroup(
                    arm.getPositionCommand(Constants.ARM_HIGH_CONE),
                    intakeArm.getPositionCommand(Constants.INARM_HIGH_CONE)
                )
            )
        );

        // CUBE
        correspondingCommandsCube.put(
            RobotStates.RobotStatesEnum.PLACE_H,
            new SequentialCommandGroup(
                elevator.getPositionCommand(Constants.ELEVATOR_HIGH_CUBE),
                new ParallelCommandGroup(
                    arm.getPositionCommand(Constants.ARM_HIGH_CUBE),
                    intakeArm.getPositionCommand(Constants.INARM_HIGH_CUBE),
                    intakeSwivel.getPositionCommand(Constants.SWIVEL_HIGH_CUBE),
                    intake.getIntakeCommand(Constants.INTAKE_FORWARD_HOLD)
                )
            )
        );

    // PLACE MID
        // CONE
        correspondingCommandsCone.put(
            RobotStates.RobotStatesEnum.PLACE_M,
            new SequentialCommandGroup(
                elevator.getPositionCommand(Constants.ELEVATOR_MID_CONE),
                new ParallelCommandGroup(
                    arm.getPositionCommand(Constants.ARM_MID_CONE),
                    intakeArm.getPositionCommand(Constants.INARM_MID_CONE),
                    intakeSwivel.getPositionCommand(Constants.SWIVEL_MID_CONE),
                    intake.getIntakeCommand(Constants.INTAKE_BACKWARD_HOLD)
                )
            )
        );

        // CUBE 
        correspondingCommandsCube.put(
            RobotStates.RobotStatesEnum.PLACE_M,

            new SequentialCommandGroup (
                elevator.getPositionCommand(Constants.ELEVATOR_MID_CUBE),
                new ParallelCommandGroup(
                    arm.getPositionCommand(Constants.ARM_MID_CUBE),
                    intakeArm.getPositionCommand(Constants.INARM_MID_CUBE),
                    intakeSwivel.getPositionCommand(Constants.SWIVEL_MID_CUBE),
                    intake.getIntakeCommand(Constants.INTAKE_FORWARD_HOLD)
                )    
            )
        );

    // PICK UP RAMP
        // CONE
        correspondingCommandsCone.put(
            RobotStates.RobotStatesEnum.PICK_UP_RAMP,
            new ParallelCommandGroup(
                elevator.getPositionCommand(0),
                arm.getPositionCommand(0),
                intakeArm.getPositionCommand(Constants.INARM_RAMP_CONE),
                intakeSwivel.getPositionCommand(Constants.SWIVEL_RAMP_CONE)
            )
        );

        // CUBE
        correspondingCommandsCube.put(
            RobotStates.RobotStatesEnum.PICK_UP_RAMP,
            new ParallelCommandGroup(
                elevator.getPositionCommand(0),
                arm.getPositionCommand(0),
                intakeArm.getPositionCommand(Constants.INARM_RAMP_CUBE),
                intakeSwivel.getPositionCommand(Constants.SWIVEL_RAMP_CUBE)
            )
        );

    // AUTO
        correspondingCommandsCube.put(
            RobotStates.RobotStatesEnum.PLACE_CUBE_MID_AUTO,
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    elevator.getPositionCommand(Constants.ELEVATOR_MID_CUBE),
                    intakeArm.getPositionCommand(Constants.INARM_MID_CUBE),
                    arm.getPositionCommand(Constants.ARM_MID_CUBE),
                    intakeSwivel.getPositionCommand(Constants.SWIVEL_MID_CUBE),
                    intake.getIntakeCommand(Constants.INTAKE_FORWARD_HOLD)
                ),
                intake.getIntakeCommand(Constants.INTAKE_BACKWARD),
                new ParallelCommandGroup(
                    elevator.getPositionCommand(0),
                    arm.getPositionCommand(0),
                    intakeArm.getPositionCommand(0),
                    intakeSwivel.getPositionCommand(0)
                )
            )
        );

    }

    // Return command for requested position 
    public Command getCommand(RobotStates.RobotStatesEnum state, boolean isCone) {
       return isCone ? correspondingCommandsCone.get(state) : correspondingCommandsCube.get(state);
    }
}