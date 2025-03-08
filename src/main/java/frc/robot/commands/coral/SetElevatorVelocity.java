// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coral;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


public class SetElevatorVelocity extends SequentialCommandGroup {
    /**
     * Sets the speed of the elevator, and if the velocity is positive verifies the arm
     * @param armSubsystem
     * @param elevatorSubsystem
     * @param velocity
     */
    public SetElevatorVelocity(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, double velocity) {
        
        if(velocity >= 0 && armSubsystem.getAngle().getDegrees() < 15) {
            addCommands(armSubsystem.goToAngle(Rotation2d.fromDegrees(15), isScheduled()) ,elevatorSubsystem.goToVelocity(velocity));
        } else {
            addCommands(elevatorSubsystem.goToVelocity(velocity));
        }
    }


    // this is gabe I finally feel like I did work yippeee
}
