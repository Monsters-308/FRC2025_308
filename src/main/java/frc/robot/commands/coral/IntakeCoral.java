// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coral;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;

/**
 * Intakes coral using the coral intake and finishes when coral is detected.
 * Runs back the arm to prevent movement while intaking.
 */
public class IntakeCoral extends ParallelCommandGroup {
    /**
     * Creates a new {@link IntakeCoral}.
     * @param armSubsystem The {@link ArmSubsystem} of the robot.
     * @param coralIntakeSubsystem The {@link CoralIntakeSubsystem} of the robot.
     * @param elevatorHeightSupplier Returns the current height of the elevator.
     */
    public IntakeCoral(ArmSubsystem armSubsystem, CoralIntakeSubsystem coralIntakeSubsystem, DoubleSupplier elevatorHeightSupplier) {
        super(
            coralIntakeSubsystem.intakeCoral(true),
            armSubsystem.goToVelocity(-ArmConstants.kArmIntakingSpeed)
                .onlyWhile(() -> elevatorHeightSupplier.getAsDouble() < ElevatorConstants.kElevatorMaxArmIntakeHeight)
        );
    }
}
