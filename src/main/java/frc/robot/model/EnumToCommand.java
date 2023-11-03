package frc.robot.model;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.HashMap;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeArm;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
                    arm.getPositionCommand(Constants.ARM_CARRY),
                    elevator.getPositionCommand(Constants.ELEVATOR_CARRY),
                    intakeArm.getIntakeArmCommand(Constants.IA_CARRY)
                ),
                new ParallelCommandGroup(
                    intakeSwivel.getCommand(Constants.SWIVEL_CARRY),
                    intake.getIntakeCommand(Constants.INTAKE_BACKWARD_HOLD)
                )
            )
        );

        // CUBE
        correspondingCommandsCube.put( 
            RobotStates.RobotStatesEnum.CARRY,
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    arm.getPositionCommand(Constants.ARM_CARRY),
                    elevator.getPositionCommand(Constants.ELEVATOR_CARRY),
                    intakeArm.getIntakeArmCommand(Constants.IA_CARRY)
                ),
                new ParallelCommandGroup(
                    intakeSwivel.getCommand(Constants.SWIVEL_CARRY),
                    intake.getIntakeCommand(Constants.INTAKE_FORWARD_HOLD)
                )
            )
        );


    // PICK UP LOW
        // CONE
        correspondingCommandsCone.put(
            RobotStates.RobotStatesEnum.PICK_UP_FLOOR,
            new ParallelCommandGroup(
                elevator.getPositionCommand(Constants.ELEVATOR_FLOOR),
                arm.getPositionCommand(0),
                intakeArm.getIntakeArmCommand(Constants.IA_CONE_FLOOR),
                intakeSwivel.getCommand(Constants.S_CONE_FLOOR) 
            )
        );

        // CUBE --> not tuned for new intake
        correspondingCommandsCube.put(
            RobotStates.RobotStatesEnum.PICK_UP_FLOOR,
            new ParallelCommandGroup(
                elevator.getPositionCommand(Constants.ELEVATOR_FLOOR),
                arm.getPositionCommand(0),
                intakeArm.getIntakeArmCommand(0) 
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
                intakeArm.getIntakeArmCommand(0),
                elevator.getPositionCommand(0),
                arm.getPositionCommand(0),
                intakeSwivel.getCommand(0)
            )
        );

        // CUBE
        correspondingCommandsCube.put(
            RobotStates.RobotStatesEnum.RETRACT_W_CARRY,
            new ParallelCommandGroup(
                intakeArm.getIntakeArmCommand(0),
                elevator.getPositionCommand(0),
                arm.getPositionCommand(0),
                intakeSwivel.getCommand(0)
            )
        );

    // PLACE HIGH
        // CONE
        correspondingCommandsCone.put(
            RobotStates.RobotStatesEnum.PLACE_H,
            new SequentialCommandGroup(
                elevator.getPositionCommand(1700), // 2000
                new ParallelCommandGroup(
                    intake.getIntakeCommand(Constants.INTAKE_BACKWARD_HOLD),
                    intakeSwivel.getCommand(5385)
                ),
                new ParallelCommandGroup(
                    arm.getPositionCommand(-6362), //-3400
                    intakeArm.getIntakeArmCommand(-38290)
                )
            )
        );

        // CUBE
        correspondingCommandsCube.put(
            RobotStates.RobotStatesEnum.PLACE_H,
            new SequentialCommandGroup(
                elevator.getPositionCommand(1607), //2000
                new ParallelCommandGroup(
                    intakeSwivel.getCommand(2545 - 6500),
                    intake.getIntakeCommand(Constants.INTAKE_FORWARD_HOLD),
                    arm.getPositionCommand(-5150), //-3400
                    intakeArm.getIntakeArmCommand(-52700)
                )
            )
        );

    // PLACE MID
        // CONE
        correspondingCommandsCone.put(
            RobotStates.RobotStatesEnum.PLACE_M,
            new SequentialCommandGroup(
                elevator.getPositionCommand(1500),
                new ParallelCommandGroup(
                    intake.getIntakeCommand(Constants.INTAKE_BACKWARD_HOLD),
                    intakeArm.getIntakeArmCommand(-45758),
                    intakeSwivel.getCommand(-812),
                    arm.getPositionCommand(-2923)
                )
            )
        );

        // CUBE 
        correspondingCommandsCube.put(
            RobotStates.RobotStatesEnum.PLACE_M,

            new SequentialCommandGroup (
                elevator.getPositionCommand(761),
                new ParallelCommandGroup(
                    intake.getIntakeCommand(Constants.INTAKE_FORWARD_HOLD),
                    intakeArm.getIntakeArmCommand(-44990),
                    intakeSwivel.getCommand(2793),
                    arm.getPositionCommand(-1719)
                )    
            )
        );

    // PICK UP RAMP
        // CONE
        correspondingCommandsCone.put(
            RobotStates.RobotStatesEnum.PICK_UP_RAMP,
            new ParallelCommandGroup(
                elevator.getPositionCommand(0),
                intakeArm.getIntakeArmCommand(-21465),
                arm.getPositionCommand(0),
                intakeSwivel.getCommand(4245)
            )
        );

        // CUBE
        correspondingCommandsCube.put(
            RobotStates.RobotStatesEnum.PICK_UP_RAMP,
            new ParallelCommandGroup(
                elevator.getPositionCommand(0),
                intakeArm.getIntakeArmCommand(-18385),
                arm.getPositionCommand(0),
                intakeSwivel.getCommand(1900)
            )
        );

    // AUTO
        correspondingCommandsCube.put(
            RobotStates.RobotStatesEnum.PLACE_CUBE_MID_AUTO,
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    intake.getIntakeCommand(0.07),
                    elevator.getPositionCommand(566),
                    intakeArm.getIntakeArmCommand(-44990),
                    intakeSwivel.getCommand(2793),
                    arm.getPositionCommand(-1719)
                ),
                intake.getIntakeCommand(-0.85),
                new ParallelCommandGroup(
                    elevator.getPositionCommand(0),
                    intakeArm.getIntakeArmCommand(0),
                    intakeSwivel.getCommand(0),
                    arm.getPositionCommand(0)
                )
            )
        );

    }

    // Return command for requested position 
    public Command getCommand(RobotStates.RobotStatesEnum state, boolean isCone) {
       return isCone ? correspondingCommandsCone.get(state) : correspondingCommandsCube.get(state);
    }
}