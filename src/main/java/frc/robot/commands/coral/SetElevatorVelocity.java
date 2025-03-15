// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coral;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Sets the speed of the elevator and, if the velocity is positive, verifies the arm's angle is greater than 15 degrees.
 * If it is not, it will move the arm to 15 degrees.
 */
public class SetElevatorVelocity extends SequentialCommandGroup {
    /**
     * Creates a new {@link SetElevatorVelocity} that sets the velocity of the elevator.
     * @param armSubsystem The {@link ArmSubsystem} of the robot.
     * @param elevatorSubsystem The {@link ElevatorSubsystem} of the robot.
     * @param velocity The velocity to set the elevator to.
     */
    public SetElevatorVelocity(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, double velocity) {
        if (velocity >= 0 && armSubsystem.getAngle().getDegrees() < 15) {
            addCommands(armSubsystem.goToAngle(Rotation2d.fromDegrees(15), false) ,elevatorSubsystem.goToVelocity(velocity));
        } else {
            addCommands(elevatorSubsystem.goToVelocity(velocity));
        }
    }
}
