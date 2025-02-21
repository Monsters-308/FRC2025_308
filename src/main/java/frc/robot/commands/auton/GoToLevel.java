package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


/*
 * This command will set the elevator to the 4th level.
 */
public class GoToLevel extends SequentialCommandGroup {
    
    public GoToLevel(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, int index) {

        super(armSubsystem.goToLevel(index, false), elevatorSubsystem.goToLevel(index, false));

    }

}