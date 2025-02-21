// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

/**
 * Subsystem that controls the algae intake.
 */
public class AlgaeIntakeRollerSubsystem extends SubsystemBase {
    /** The {@link SparkMax} controller for the algae intake roller. */
    private final SparkMax m_algaeIntakeRoller = new SparkMax(AlgaeIntakeConstants.kAlgaeIntakeArmMotorCanId, MotorType.kBrushless);


    /** Constructs an {@link AlageIntakeSubsystem} that controls the algae intake. */
    public AlgaeIntakeRollerSubsystem() {
        SparkMaxConfig rollerMotorConf = new SparkMaxConfig();
        
        // sets the configation of the motors
        rollerMotorConf
            .inverted(AlgaeIntakeConstants.kAlgaeIntakeRollerInverted)
            .idleMode(AlgaeIntakeConstants.kAlgaeIntakeRollerIdleMode)
            .smartCurrentLimit(AlgaeIntakeConstants.kAlageIntakeRollerSmartCurrentLimit);

        m_algaeIntakeRoller.configure(rollerMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the speed of the roller.
     * @param speed The speed to set.
     */
    public void setRollerSpeed(double speed) {
        m_algaeIntakeRoller.set(speed);
    }

    /**
     * Creates a <code>Command</code> that intakes algae.
     * @return The <code>Command</code> object.
     * @see Command
     */
    public Command intakeAlgae() {
        return runEnd(
            () -> setRollerSpeed(AlgaeIntakeConstants.kRollerSpeed),
            () -> setRollerSpeed(0)
        );
    }

    /**
     * Creates a <code>Command</code> that shoots algae.
     * @return The <code>Command</code> object.
     * @see Command
     */
    public Command shootAlgae() {
        return runEnd(
            () -> setRollerSpeed(-AlgaeIntakeConstants.kRollerSpeed),
            () -> setRollerSpeed(0)
        );
    }
}
