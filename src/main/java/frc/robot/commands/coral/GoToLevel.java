package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


/**
 * Moves the elevator and arm to the specified level.
 */
public class GoToLevel extends SequentialCommandGroup {
    /**
     * Moves the elevator and arm to the specified level.
     * @param armSubsystem The {@link ArmSubsystem} of the robot.
     * @param elevatorSubsystem The {@link ELevatorSubsystem} of the robot.
     * @param index The index of the level to move the elevator and arm to.
     */
    public GoToLevel(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, int index) {
        super(armSubsystem.goToLevel(index, false), elevatorSubsystem.goToLevel(index, false));
    }

}