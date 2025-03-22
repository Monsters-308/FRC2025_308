// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
        if (index == 0) {
            addCommands(elevatorSubsystem.goToLevel(index), armSubsystem.goToLevel(index));
        } else {
            addCommands(
                armSubsystem.goToLevel(index, true), 
                new WaitUntilCommand(() -> armSubsystem.getAngle().getDegrees() > 15),
                elevatorSubsystem.goToLevel(index)
            );
        }

    }
}