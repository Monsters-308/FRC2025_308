// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AlgaeIntakeConstants;

/**
 * Subsystem that controls the algae intake.
 */
public class AlgaeIntakeArmSubsystem extends SubsystemBase {
    /** The {@link SparkMax} controller for the algae intake arm. */
    private final SparkMax m_algaeIntakeArm = new SparkMax(AlgaeIntakeConstants.kAlgaeIntakeRollerMotorCanId, MotorType.kBrushless);

    /** A {@link RelativeEncoder} that measures the position of the alage intake arm. */
    private final RelativeEncoder m_algaeIntakeArmEncoder;

    /** Constructs an {@link AlageIntakeSubsystem} that controls the algae intake. */
    public AlgaeIntakeArmSubsystem() {
        SparkMaxConfig armMotorConf = new SparkMaxConfig();
        
        // sets the configation of the motors
        armMotorConf
            .inverted(AlgaeIntakeConstants.kAlgaeIntakeArmInverted)
            .idleMode(AlgaeIntakeConstants.kAlgaeIntakeArmIdleMode)
            .smartCurrentLimit(AlgaeIntakeConstants.kAlageIntakeArmSmartCurrentLimit);

        m_algaeIntakeArm.configure(armMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_algaeIntakeArmEncoder = m_algaeIntakeArm.getEncoder();
    }

    
    /**
     * Sets the speed of the arm.
     * @param speed The speed to set.
     */
    public void setArmSpeed(double speed) {
        m_algaeIntakeArm.set(speed);
    }


    /**
     * Creates a <code>Command</code> that brings in the algae intake arm.
     * @return The <code>Command</code> object.
     * @see Command
     */
    public Command armIn() {
        return runOnce(() -> setArmSpeed(-AlgaeIntakeConstants.kArmSpeed))
            .andThen(new WaitCommand(AlgaeIntakeConstants.kArmTimeout))
            .finallyDo(() -> setArmSpeed(0));
    }

    /**
     * Creates a <code>Command</code> that brings in the algae intake out.
     * @return The <code>Command</code> object.
     * @see Command
     */
    public Command armOut() {
        return runOnce(() -> setArmSpeed(AlgaeIntakeConstants.kArmSpeed))
            .andThen(new WaitCommand(AlgaeIntakeConstants.kArmTimeout))
            .finallyDo(() -> setArmSpeed(0));
    }

    /**
     * Creates a <code>Command</code> that toggles the position of the algae intake arm.
     * @return The <code>Command</code> object.
     * @see Command
     */
    public Command armToggle() {
        return m_algaeIntakeArmEncoder.getPosition() < AlgaeIntakeConstants.kArmErrorThreshold ? armOut() : armIn();
    }
}