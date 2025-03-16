// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;

/**
 * Intakes coral using the coral intake and finishes when coral is detected.
 * Runs back the arm to prevent movement while intaking.
 */
public class IntakeCoral extends ParallelDeadlineGroup {
    /**
     * Creates a new {@link IntakeCoral}.
     * @param armSubsystem The {@link ArmSubsystem} of the robot.
     * @param coralIntakeSubsystem The {@link CoralIntakeSubsystem} of the robot.
     */
    public IntakeCoral(ArmSubsystem armSubsystem, CoralIntakeSubsystem coralIntakeSubsystem) {
        super(coralIntakeSubsystem.intakeCoral(true));
    }
}
