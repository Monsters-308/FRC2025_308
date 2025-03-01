// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.LoggingUtils;
import frc.robot.utils.Utils;

/**
 * Subsystem that controls the coral arm of the robot.
 */
public class ArmSubsystem extends SubsystemBase {
    /** The motor controller for the coral arm. */
    public final SparkMax m_armMotor = new SparkMax(ArmConstants.kArmMotorCanId, MotorType.kBrushless);
    /** The encoder for measuring the position and velocity of the motor. */
    private final AbsoluteEncoder m_armEncoder;

    /** The {@link ProfiledPIDController} for the arm motor. */
    private final ProfiledPIDController m_angleController = new ProfiledPIDController(
        ArmConstants.kArmP,
        ArmConstants.kArmI,
        ArmConstants.kArmD,
        new TrapezoidProfile.Constraints(
            ArmConstants.kArmMaxSpeedRPS,
            ArmConstants.kArmMaxAccelerationRPSSquared
        )
    );

    /** The {@link ArmFeedforward} for the arm motor. */
    // private final ArmFeedforward m_armFeedforward = new ArmFeedforward(
    //     ArmConstants.kArmS,
    //     ArmConstants.kArmG,
    //     ArmConstants.kArmV,
    //     ArmConstants.kArmA
    // );

    /** Whether to use PID or not. */
    private boolean m_isPIDMode = true;

    /** A {@link SuffleboardTab} to write arm properties to the dashboard. */
    private final ShuffleboardTab m_armTab = Shuffleboard.getTab("Arm");

    private final GenericEntry m_gravityEntry;

    private double m_speed = 0;

    /**
     * Constructs an {@link ArmSubsystem} that controls the coral arm of the robot.
     */
    public ArmSubsystem() {
        SparkMaxConfig armMotorConf = new SparkMaxConfig();
        armMotorConf
            .inverted(ArmConstants.kArmMotorInverted)
            .smartCurrentLimit(ArmConstants.kSmartCurrentLimit)
            .idleMode(ArmConstants.kIdleMode);
        armMotorConf.absoluteEncoder
            .positionConversionFactor(ArmConstants.kPositionEncoderConversionFactor)
            .velocityConversionFactor(ArmConstants.kVelocityEncoderConversionFactor);

        m_armMotor.configure(armMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_armEncoder = m_armMotor.getAbsoluteEncoder();

        m_armTab.addDouble("Arm Angle", () -> Utils.roundToNearest(getAngle().getDegrees(), 2));
        m_armTab.addDouble("Arm Velocity", () -> Utils.roundToNearest(getVelocity().getDegrees(), 2));

        m_armTab.addDouble("Arm Angle Setpoint", () -> 
            Utils.roundToNearest(Units.rotationsToDegrees(m_angleController.getSetpoint().position), 2));
        m_armTab.addDouble("Arm Velocity Setpoint", () -> 
            Utils.roundToNearest(Units.rotationsToDegrees(m_angleController.getSetpoint().velocity), 2));

        m_armTab.addDouble("Arm Angle Goal", () -> 
            Utils.roundToNearest(Units.rotationsToDegrees(m_angleController.getGoal().position), 2));
        m_armTab.addDouble("Arm Velocity Goal", () -> 
            Utils.roundToNearest(Units.rotationsToDegrees(m_angleController.getGoal().velocity), 2));

        m_gravityEntry = m_armTab.addPersistent("Gravity Offset", ArmConstants.kArmG).getEntry();

        LoggingUtils.logSparkMax(m_armMotor);

        Utils.configureSysID(
            m_armTab.getLayout("Arm SysID", BuiltInLayouts.kList), this, 
            () -> m_isPIDMode = false,
            voltage -> {
                m_armMotor.setVoltage(voltage);
            }
        );

        m_angleController.enableContinuousInput(0, 1);
        m_angleController.setTolerance(ArmConstants.kArmTolerance);
    }

    /**
     * Resets and sets the goal of the angle PID controller.
     * @param angle The angle to set as a {@link Rotation2d}.
     */
    public void setAngle(Rotation2d angle) {
        m_isPIDMode = true;

        Rotation2d constrainedAngle = Rotation2d.fromDegrees(Utils.angleConstrain(angle.getDegrees()));
        m_angleController.reset(
            getAngle().getRotations(),
            getVelocity().getRotations()
        );

        m_angleController.setGoal(constrainedAngle.getRotations());
    }

    /**
     * Creates a command the moves the arm to the specified angle.
     * @param angle The angle the arm should move to as a {@link Rotation2d}.
     * @param endImmediately Whether the command should end immediately or wait until the elevator has reached the angle.
     * @return The runnable command.
     */
    public Command goToAngle(Rotation2d angle, boolean endImmediately) {
        return runOnce(() -> setAngle(angle))
            .andThen(new WaitUntilCommand(() -> m_angleController.atGoal() || endImmediately));
    }

    /**
     * Creates a command the moves the arm to the specified angle.
     * @param index The index of the level the arm should move to.
     * @param endImmediately Whether the command should end immediately or wait until the elevator has reached the angle.
     * @return The runnable command.
     */
    public Command goToLevel(int index, boolean endImmediately) {
        return goToAngle(ArmConstants.kArmLevelAngles[index], endImmediately);
    }

    /**
     * Gets the current angle of the coral arm.
     * @return The current angle of the arm.
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(1 - m_armEncoder.getPosition());
    }

    /**
     * Gets the current velocity of the arm.
     * @return The velocity of the arm as a {@link Rotation2d} object.
     */
    public Rotation2d getVelocity() {
        return Rotation2d.fromRotations(m_armEncoder.getVelocity());
    }

    public void setSpeed(double speed) {
        m_isPIDMode = false;
        m_speed = speed;
        m_armMotor.set(m_speed);
    }

    public Command goToSpeed(double speed) {
        return runOnce(() -> setSpeed(speed))
            .andThen(run(() -> {}))
            .finallyDo(() -> m_isPIDMode = true);
    }

    /**
     * Stops movement of the coral arm.
     */
    public void stop() {
        setAngle(getAngle());
    }

    @Override
    public void periodic() {
        if (m_isPIDMode) {
            // State setpoint = m_angleController.getSetpoint();
            // double velocitySetpoint = Units.rotationsToRadians(setpoint.velocity);

            m_armMotor.set(
                -m_angleController.calculate(getAngle().getRotations()) +
                getAngle().getSin() * m_gravityEntry.getDouble(ArmConstants.kArmG)
                // m_armFeedforward.calculateWithVelocities(getAngle().getRadians(), getVelocity().getRadians(), velocitySetpoint)
            );
        }
    }
}