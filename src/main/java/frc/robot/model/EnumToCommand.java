package frc.robot.model;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeArm;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSwivel;

public class EnumToCommand {
    HashMap<RobotStates.RobotStatesEnum, Command> corresponding_commands_cube = new HashMap<RobotStates.RobotStatesEnum, Command>();
    HashMap<RobotStates.RobotStatesEnum, Command> corresponding_commands_cone = new HashMap<RobotStates.RobotStatesEnum, Command>();

    
    public EnumToCommand(Elevator elevator, Arm arm, Intake intake, IntakeArm intakearm, IntakeSwivel intakeswivel){    
        // CARRY

        // CONE

        corresponding_commands_cone.put( 
            RobotStates.RobotStatesEnum.CARRY,
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    arm.getPositionCommand(Constants.ARM_RETRACT),
                    elevator.getPositionCommand(Constants.ELEVATOR_LOW),
                    intakearm.getIntakeArmCommand(Constants.VTICKS)
                ),
                intakearm.getIntakeArmCommand(0)
            )
        );

        // CUBE

        corresponding_commands_cube.put( 
            RobotStates.RobotStatesEnum.CARRY,
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    arm.getPositionCommand(Constants.ARM_RETRACT),
                    elevator.getPositionCommand(Constants.ELEVATOR_LOW),
                    intakearm.getIntakeArmCommand(Constants.VTICKS)
                ),
                intakearm.getIntakeArmCommand(0)
            )
        );


        // PICK UP LOW

        // CONE

        corresponding_commands_cone.put(
            RobotStates.RobotStatesEnum.PICK_UP_LOW,
            new ParallelCommandGroup(
                elevator.getPositionCommand(0),
                arm.getPositionCommand(0),
                intakearm.getIntakeArmCommand(Constants.HTICKS + 12000) //9500
            )
        );

        // CUBE

        corresponding_commands_cube.put(
            RobotStates.RobotStatesEnum.PICK_UP_LOW,
            new ParallelCommandGroup(
                elevator.getPositionCommand(0),
                arm.getPositionCommand(0),
                intakearm.getIntakeArmCommand(Constants.HTICKS + 12000) //9500
            )
        );

        // INTAKE ON

        corresponding_commands_cone.put(
            RobotStates.RobotStatesEnum.INTAKEON,
            intake.get_intakeCommand(-0.85)
        );

        corresponding_commands_cube.put(
            RobotStates.RobotStatesEnum.INTAKEON,
            intake.get_intakeCommand(-0.85)
        );

        // INTAKE OFF

        corresponding_commands_cone.put(
            RobotStates.RobotStatesEnum.INTAKEOFF,
            intake.get_intakeCommand(0.0)
        );

        corresponding_commands_cube.put(
            RobotStates.RobotStatesEnum.INTAKEOFF,
            intake.get_intakeCommand(0.0)
        );


        // RETRACT W CARRY

        corresponding_commands_cone.put(
            RobotStates.RobotStatesEnum.RETRACT_W_CARRY,
            new ParallelCommandGroup(
                intakearm.getIntakeArmCommand(0),
                elevator.getPositionCommand(Constants.ELEVATOR_LOW),
                arm.getPositionCommand(0)
            )
        );

        corresponding_commands_cube.put(
            RobotStates.RobotStatesEnum.RETRACT_W_CARRY,
            new ParallelCommandGroup(
                intakearm.getIntakeArmCommand(0),
                elevator.getPositionCommand(Constants.ELEVATOR_LOW),
                arm.getPositionCommand(0)
            )
        );

        // PLACE HIGH

        // CONE

        corresponding_commands_cone.put(
            RobotStates.RobotStatesEnum.PLACE_H,
            new SequentialCommandGroup(
                elevator.getPositionCommand(2000),
                new ParallelCommandGroup(
                    arm.getPositionCommand(-3400),
                    intakearm.getIntakeArmCommand(-65000)
                )
            )
        );

        // CUBE

        corresponding_commands_cube.put(
            RobotStates.RobotStatesEnum.PLACE_H,
            new SequentialCommandGroup(
                elevator.getPositionCommand(2000),
                new ParallelCommandGroup(
                    arm.getPositionCommand(-3400),
                    intakearm.getIntakeArmCommand(-65000)
                )
            )
        );

        corresponding_commands_cone.put(
            RobotStates.RobotStatesEnum.PLACE_CONE_AUTO,

            new SequentialCommandGroup(
                intakearm.getIntakeArmCommand(Constants.VTICKS),

                new ParallelCommandGroup(
                    elevator.getPositionCommand(2000),
                    arm.getPositionCommand(-3200)
                ),

                new SequentialCommandGroup(
                    intakearm.getIntakeArmCommand(-63000),
                    new WaitCommand(1.5),
                    intake.get_intakeCommand(-0.85),
                    intakearm.getIntakeArmCommand(Constants.VTICKS)                
                ),

                new ParallelCommandGroup(
                    intakearm.getIntakeArmCommand(Constants.VTICKS),
                    arm.getPositionCommand(Constants.ARM_RETRACT)
                ),


                new ParallelCommandGroup(
                    elevator.getPositionCommand(Constants.ELEVATOR_LOW),
                    intakearm.getIntakeArmCommand(0)
                )
            )
        );


        corresponding_commands_cube.put(
            RobotStates.RobotStatesEnum.PLACE_CUBE_AUTO,

            new SequentialCommandGroup(
                intakearm.getIntakeArmCommand(Constants.VTICKS),

                new ParallelCommandGroup(
                    elevator.getPositionCommand(2000),
                    arm.getPositionCommand(-3400)
                ),

                new SequentialCommandGroup(
                    intakearm.getIntakeArmCommand(-58000),
                    new WaitCommand(1),
                    intake.get_intakeCommand(0.85),
                    intakearm.getIntakeArmCommand(Constants.VTICKS)                
                ),

                new ParallelCommandGroup(
                    intakearm.getIntakeArmCommand(Constants.VTICKS),
                    arm.getPositionCommand(Constants.ARM_RETRACT)
                ),


                new ParallelCommandGroup(
                    elevator.getPositionCommand(Constants.ELEVATOR_LOW),
                    intakearm.getIntakeArmCommand(0)
                )
            )
        );

        // PLACE M

        // CONE

        corresponding_commands_cone.put(
            RobotStates.RobotStatesEnum.PLACE_M,
            new ParallelCommandGroup(
                elevator.getPositionCommand(2000),
                intakearm.getIntakeArmCommand(-87000)
            )
        );

        // CUBE 

        corresponding_commands_cube.put(
            RobotStates.RobotStatesEnum.PLACE_M,
            new ParallelCommandGroup(
                elevator.getPositionCommand(2000),
                intakearm.getIntakeArmCommand(-87000)
            )
        );

        // PICK UP RAMP

        // CONE

        corresponding_commands_cone.put(
            RobotStates.RobotStatesEnum.PICK_UP_RAMP,
            new ParallelCommandGroup(
                elevator.getPositionCommand(Constants.ELEVATOR_LOW - 200),
                intakearm.getIntakeArmCommand(Constants.VTICKS - 7000),
                arm.getPositionCommand(0)
            )
        );

        // CUBE

        corresponding_commands_cube.put(
            RobotStates.RobotStatesEnum.PICK_UP_RAMP,
            new ParallelCommandGroup(
                elevator.getPositionCommand(Constants.ELEVATOR_LOW - 200),
                intakearm.getIntakeArmCommand(Constants.VTICKS - 7000),
                arm.getPositionCommand(0)
            )
        );


        corresponding_commands_cone.put(
            RobotStates.RobotStatesEnum.PLACE_CUBE_LOW_AUTO,

            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    elevator.getPositionCommand(0),
                    arm.getPositionCommand(0),
                    intakearm.getIntakeArmCommand(Constants.HTICKS + 12000)
                ),

                intake.get_intakeCommand(0.85),
                intakearm.getIntakeArmCommand(0)
            )
        );

    }

    public Command getCommand(RobotStates.RobotStatesEnum state, boolean is_cone) {
       return is_cone ? corresponding_commands_cone.get(state) : corresponding_commands_cube.get(state);
    }
}
