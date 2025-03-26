// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.LoggingUtils;
import frc.robot.utils.Utils;

/**
 * Subsystem that controls the robot's elevator.
 */
public class ElevatorSubsystem extends SubsystemBase {
    /** The leader motor controller of the elevator. */
    private final SparkMax m_elevatorMotor = new SparkMax(
        ElevatorConstants.kElevatorMotorCanId,
        MotorType.kBrushless);

    /** Encoder that represents the elevator motors. */
    private final RelativeEncoder m_elevatorEncoder;

    /** PID controller the controls the elevator motors. */
    private final ProfiledPIDController m_elevatorPIDController = new ProfiledPIDController(
        ElevatorConstants.kElevatorP,
        ElevatorConstants.kElevatorI,
        ElevatorConstants.kElevatorD,
        new TrapezoidProfile.Constraints(
            ElevatorConstants.kElevatorMaxSpeedInchesPerSecond,
            ElevatorConstants.kElevatorMaxAccelerationInchesPerSecondSquared
        )
    );

    // private final ElevatorFeedforward m_elevatorFeedforward = new ElevatorFeedforward(
    //     ElevatorConstants.kElevatorS,
    //     ElevatorConstants.kElevatorG,
    //     ElevatorConstants.kElevatorV,
    //     ElevatorConstants.kElevatorA
    // );

    /** A limit switch that triggers when the elevator reaches the bottom. */
    private final DigitalInput m_bottomSwitch = new DigitalInput(ElevatorConstants.kBottomSwitchChannel);

    /** A backup limit switch that triggers when the elevator reaches the bottom. */
    private final DigitalInput m_backupBottomSwitch = new DigitalInput(ElevatorConstants.kBackupBottomSwitchChannel);

    private final DigitalInput m_topSwitch = new DigitalInput(ElevatorConstants.kTopSwitchChannel);

    /** A {@link ShuffleboardTab} to write elevator properties to the dashboard. */
    private final ShuffleboardTab m_elevatorTab = Shuffleboard.getTab("Elevator");
    /** A {@link ShuffleboardLayout} that holds the go to level command. */
    private final ShuffleboardLayout m_goToHeightLayout = m_elevatorTab.getLayout("Go To Height", BuiltInLayouts.kList);
    /** The {@link GenericEntry} that contains the level to send the robot to when the dashboard button is pressed. */
    private GenericEntry m_goToHeightEntry = null;

    /** Whether or not the elevator is currently using PID or setting the speed directly. */
    private boolean m_isPIDMode = false;

    /**
     * Initializes an ElevatorSubsystem to control the robot's elevator.
     */
    public ElevatorSubsystem() {
        final SparkMaxConfig leaderMotorConfig = new SparkMaxConfig();

        leaderMotorConfig
            .idleMode(ElevatorConstants.kElevatorIdleMode)
            .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
            .inverted(ElevatorConstants.kElevatorMotorInverted);

        leaderMotorConfig.encoder
            .positionConversionFactor(ElevatorConstants.kElevatorEncoderPositionFactor)
            .velocityConversionFactor(ElevatorConstants.kElevatorEncoderVelocityFactor);

        m_elevatorMotor.configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        LoggingUtils.logSparkMax(m_elevatorMotor);

        m_elevatorEncoder = m_elevatorMotor.getEncoder();

        m_elevatorTab.addDouble("Elevator Height", () -> Utils.roundToNearest(getElevatorHeight(), 2));
        m_elevatorTab.addInteger("Elevator Level", this::getCurrentLevel);
        m_elevatorTab.addDouble("Elevator Speed", () -> Utils.roundToNearest(getElevatorVelocity(), 2));

        m_elevatorTab.add("Zero Encoder", zeroEncoder());

        m_goToHeightLayout.add(
            new DeferredCommand(() -> 
                goToHeight(m_goToHeightEntry.getDouble(0)),
                new HashSet<>(Arrays.asList(this))
            )
        );

        m_goToHeightEntry = m_goToHeightLayout.add("Height", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of(
                "Min", 0,
                "Max", ElevatorConstants.kElevatorMaxHeight
            )).getEntry();

        Utils.configureSysID(
            m_elevatorTab.getLayout("Elevator SysID", BuiltInLayouts.kList), this, 
            voltage -> {
                m_isPIDMode = false;
                m_elevatorMotor.setVoltage(voltage);
            }
        );

        m_elevatorTab.addBoolean("Is At Bottom", () -> isAtBottom());
        m_elevatorTab.addBoolean("Is At Top", () -> m_topSwitch.get());
    }

    /**
     * Resets and sets the height goal of the PID controller.
     * @param height The height goal to set.
     */
    public void setElevatorHeight(double height) {
        height = MathUtil.clamp(height, 0, ElevatorConstants.kElevatorMaxHeight);

        m_elevatorPIDController.reset(
            getElevatorHeight(),
            getElevatorVelocity()
        );

        m_elevatorPIDController.setGoal(height);

        m_isPIDMode = true;
    }

    /**
     * Sets the height goal of the PID controller to the specified level.
     * @param index The index of the level to set.
     */
    public void setElevatorLevel(int index) {
        setElevatorHeight(ElevatorConstants.kElevatorLevelHeights[index]);
    }

    /**
     * Creates a {@link Command} that moves to elevator to the specified level.
     * @param index The index of the level.
     * @return The runnable <code>Command</code>.
     */
    public Command goToLevel(int index) {
        return goToLevel(index, false);
    }

    /**
     * Creates a {@link Command} that moves to elevator to the specified level.
     * @param index The index of the level.
     * @param endImmediately Whether the command should end immediately or wait until the elevator has reached the level.
     * @return The runnable <code>Command</code>.
     */
    public Command goToLevel(int index, boolean endImmediately) {
        return goToHeight(
            ElevatorConstants.kElevatorLevelHeights[index],
            endImmediately
        );
    }

    /**
     * Creates a {@link Command} that moves the elevator to the specified height.
     * @param height The height to move the elevator to in meters.
     * @return The runnable <code>Command</code>.
     */
    public Command goToHeight(double height) {
        return goToHeight(height, false);
    }

    /**
     * Creates a {@link Command} that moves the elevator to the specified height.
     * @param height The height to move the elevator to in meters.
     * @param endImmediately Whether the command should end immediately or wait until the elevator has reached the height.
     * @return The runnable <code>Command</code>.
     */
    public Command goToHeight(double height, boolean endImmediately) {
        if (height == 0) {
            return runOnce(() -> setElevatorHeight(height))
            .andThen(new WaitUntilCommand(() -> m_elevatorPIDController.atGoal() | endImmediately))
            .andThen(zeroElevator())
            .withName("Go");
        }
        
        return runOnce(() -> setElevatorHeight(height))
            .andThen(new WaitUntilCommand(() -> 
                m_elevatorPIDController.atGoal() || endImmediately
            ))
            .withName("Go");
    }

    /**
     * Creates a {@link Command} that sets the velocity of the elevator.
     * @param velocity The velocity to set the elevator to.
     * @return The runnable <code>Command</code>.
     */
    public Command goToVelocity(double velocity) {
        return runOnce(() -> setElevatorVelocity(velocity));
    }

    /**
     * Creates a {@link Command} that sets the position of the elevator encoder to zero.
     * @return The runnable <code>Command</code>.
     */
    public Command zeroEncoder() {
        return runOnce(() -> m_elevatorEncoder.setPosition(0)).ignoringDisable(true);
    }

    /**
     * Sets the velocity of the elevator.
     * @param velocity The velocity to set the motor to from <code>-1</code> to <code>1</code>.
     */
    public void setElevatorVelocity(double velocity) {
        m_isPIDMode = false;

        velocity += ElevatorConstants.kElevatorG;

        if (isAtBottom()) {
            // Prevent the elevator from going down when it reaches the bottom
            // by preventing the speed from being negative
            velocity = Math.max(0, velocity);
        } else if (getElevatorHeight() >= ElevatorConstants.kElevatorMaxHeight) {
            // Prevent the elevator from going up when it reaches the top
            // by preventing the speed from being positive
            velocity = Math.min(0, velocity);
        }

        m_elevatorMotor.set(velocity);
    }

    /** 
     * Gets the current closest level to the elevator.
     * @return The index of the level.
     */
    public int getCurrentLevel() {
        double height = getElevatorHeight();
        Double smallestDifference = null;
        int index = 0;

        for (int i = 0; i < ElevatorConstants.kElevatorLevelHeights.length; i++) {
            double difference = Math.abs(ElevatorConstants.kElevatorLevelHeights[i] - height);

            // Check if current height difference is smaller than the previous one or if there is no previous one
            if (smallestDifference == null || difference < smallestDifference) {
                smallestDifference = difference; // Sets new smallest difference if it is
                index = i; // Sets level index to i because it has a smaller difference
            }
        }

        return index;
    }

    /**
     * Gets the current height of the elevator in meters.
     * @return The height of the elevator.
     */
    public double getElevatorHeight() {
        return m_elevatorEncoder.getPosition();
    }

    /**
     * Gets the current velocity of the elevator.
     * @return The velocity of the elevator in meters per second.
     */
    public double getElevatorVelocity() {
        return m_elevatorEncoder.getVelocity();
    }

    /**
     * Stops the elevator.
     */
    public void stopElevator() {
        setElevatorHeight(getElevatorHeight());
    }

    /**
     * Creates a {@link Command} Moves the elevator down until it touches the magnetic sensor.
     * @returns The runnable <code>Command</code>
     */
    public Command zeroElevator() {
        return goToVelocity(-ElevatorConstants.kElevatorManualSpeed)
            .andThen(new WaitUntilCommand(() -> isAtBottom()))
            .finallyDo(() -> stopElevator())
            .withTimeout(0.5);
    }

    /**
     * Gets whether or not the elevator is at its lowest point.
     * @return The boolean value.
     */
    public boolean isAtBottom() {
        return m_bottomSwitch.get() || m_backupBottomSwitch.get();
    }

    @Override
    public void periodic() {
        final double currentHeight = getElevatorHeight();

        if (m_isPIDMode) {
            // double velocitySetpoint = m_elevatorPIDController.getSetpoint().velocity;
            
            m_elevatorMotor.set(
                m_elevatorPIDController.calculate(currentHeight) + ElevatorConstants.kElevatorG
                // m_elevatorFeedforward.calculateWithVelocities(getElevatorVelocity(), velocitySetpoint)
            );
        }

        if (isAtBottom()) {
            // Prevent the elevator from going down when it reaches the bottom
            // by preventing the speed from being negative
            m_elevatorMotor.set(Math.max(0, m_elevatorMotor.get()));
            m_elevatorEncoder.setPosition(0);
        } else if (currentHeight >= ElevatorConstants.kElevatorMaxHeight) {
            // Prevent the elevator from going up when it reaches the top
            // by preventing the speed from being positive
            m_elevatorMotor.set(Math.min(0, m_elevatorMotor.get()));
            m_elevatorEncoder.setPosition(ElevatorConstants.kElevatorMaxHeight);
        }
    }
}
