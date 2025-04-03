// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.utils.LoggingUtils;


/**
 * The subsystem that controls the coral intake.
 */
public class CoralIntakeSubsystem extends SubsystemBase {
    /** The motor controller for the coral intake motor. */
    private final SparkMax m_coralIntakeMotor = new SparkMax(CoralIntakeConstants.kCoralIntakeMotorCanId, MotorType.kBrushless);
    /** The senser that detects whether or not coral has left the funnel. */
    private final DigitalInput m_backSensor = new DigitalInput(CoralIntakeConstants.kBackSensorChannel);
    /** The senser that detects whether or not coral is in the intake. */
    private final DigitalInput m_sensor = new DigitalInput(CoralIntakeConstants.kSensorChannel);

    /** The {@link ShuffleboardTab} used for logging coral intake sensors. */
    private final ShuffleboardTab m_coralIntakeTab = Shuffleboard.getTab("Arm");

    /**
     * Constructs a {@link CoralIntakeSubsystem} that controls the coral intake.
     */
    public CoralIntakeSubsystem() {
        SparkMaxConfig coralIntakeMotorConf = new SparkMaxConfig();
        
        // sets the configation of the motors
        coralIntakeMotorConf
            .inverted(CoralIntakeConstants.kCoralIntakeInverted)
            .idleMode(CoralIntakeConstants.kCoralIntakeIdleMode)
            .smartCurrentLimit(CoralIntakeConstants.kSmartCurrentLimit);

        m_coralIntakeMotor.configure(coralIntakeMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_coralIntakeTab.addBoolean("Coral Detected", this::isCoralDetected);

        LoggingUtils.logSparkMax(m_coralIntakeMotor);

    }

    /**
     * sets the speed of the flywheels depending on the what you pass into the pramader.
     */
    public void setCoralSpeed(double speed) {
        m_coralIntakeMotor.set(speed);
    }

    /**
     * Gets whether or not the coral has left the funnel.
     * @returns The boolean value.
     */
    public boolean hasCoralLeftFunnel() {
        return m_backSensor.get();
    }

    /**
     * Gets whether or not the coral is in the intake.
     * @returns The boolean value.
     */
    public boolean isCoralDetected() {
        return !m_sensor.get();
    }
    
    /**
     * Creates a {@link Command} that shoots the coral.
     * @return The <code>Command</code> object.
     */
    public Command shootCoral() {
        return runOnce(() -> setCoralSpeed(CoralIntakeConstants.kCoralShootSpeed))
            .andThen(run(() -> {}))
            .finallyDo(() -> setCoralSpeed(0));
    }

    /**
     * Creates a {@link Command} that reverses the coral intake.
     * @return The <code>Command</code> object.
     */
    public Command reverseCoral() {
        return runOnce(() -> setCoralSpeed(-CoralIntakeConstants.kCoralIntakeSpeed))
            .andThen(run(() -> {}))
            .finallyDo(() -> setCoralSpeed(0));
    }

    /**
     * Creates a {@link Command} that intakes the coral.
     * @param stopWhenDetected Whether or not to stop the coral intake when coral is detected inside.
     * @return The <code>Command</code> object.
     */
    public Command intakeCoral(boolean stopWhenDetected) {
        return runOnce(() -> setCoralSpeed(CoralIntakeConstants.kCoralIntakeSpeed))
            .andThen(new WaitUntilCommand(() -> (isCoralDetected() && hasCoralLeftFunnel()) || !stopWhenDetected))
            .andThen(reverseCoral().withTimeout(0.06))
            .finallyDo(() -> {
                if (!stopWhenDetected) return;
                setCoralSpeed(0);
            });
    }

}
