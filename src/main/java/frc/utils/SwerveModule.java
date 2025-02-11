// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ModuleConstants;

/**
 * Utility class for managing the drive and turning <code>SparkMax</code> motor controllers for each wheel.
 * @see SparkMax
 */
public class SwerveModule {
    /** The driving {@link SparkMax} motor controller. */
    private final SparkMax m_drivingSparkMax;
    /** The turning {@link SparkMax} motor controller. */
    private final SparkMax m_turningSparkMax;

    /** The drive {@link RelativeEncoder} for reading position and velocity data. */
    private final RelativeEncoder m_drivingEncoder;
    /** The turning {@link RelativeEncoder} used to forward values from {@link SwerveModule#m_turningAbsoluteEncoder} to the {@link SparkMax}. */
    private final RelativeEncoder m_turningEncoder;
    /** The turning {@link CANcoder} used for reading position and velocity data. These values are forwared into {@link SwerveModule#m_turningEncoder} */
    private final CANcoder m_turningAbsoluteEncoder;

    /** The current desired angle for the wheel. */
    private double m_desiredAngle = 0;

    /** The drive {@link SparkClosedLoopController} for managing PID on the {@link SwerveModule#m_drivingSparkMax}. */
    private final SparkClosedLoopController m_drivingPIDController;
    /** The turning {@link SparkClosedLoopController} for managing PID on the {@link SwerveModule#m_turningSparkMax}. */
    private final SparkClosedLoopController m_turningPIDController;

    /** Used to detect when the turning encoders are updated since they're not updated as frequently over CAN. */
    private double m_mostRecentTurningEncoderValue = 0;

    /**
     * Constructs a Swerve Module and configures the driving and turning <code>SparkMax</code> objects,
     * the turning <code>CANcoder</code>, and <code>SparkClosedLoopController</code> PID controllers. This configuration is specific to the MK4 and 
     * MK4i swerve modules with 2 NEOs, 2 Spark Maxes, and an SRX Mag Encoder.
     * @param drivingCanId The CAN ID of the driving <code>SparkMax</code>
     * @param turningCanId The CAN ID of the turning <code>SparkMax</code>
     * @param drivingEncoderId The CAN ID of the turning <code>CANcoder</code>
     * @param invertDrive Whether or not to invert the drive <code>SparkMax</code>
     * @see SparkMax
     * @see CANcoder
     * @see SparkClosedLoopController
     * 
     */
    public SwerveModule(int drivingCanId, int turningCanId, int turningEncoderId, boolean invertDrive) {
        m_drivingSparkMax = new SparkMax(drivingCanId, MotorType.kBrushless);
        m_turningSparkMax = new SparkMax(turningCanId, MotorType.kBrushless);
        m_turningAbsoluteEncoder = new CANcoder(turningEncoderId);

        // Configure Driving motor
        SparkMaxConfig drivingConfig = new SparkMaxConfig();
        drivingConfig
            .idleMode(ModuleConstants.kDrivingMotorIdleMode)
            .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
            .inverted(invertDrive);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        drivingConfig.encoder
            .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        // Set the PID gains for the driving motor.
        drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ModuleConstants.kDrivingP)
            .i(ModuleConstants.kDrivingI)
            .d(ModuleConstants.kDrivingD)
            .velocityFF(ModuleConstants.kDrivingFF)
            .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

        // Configure driving Spark Max with configuration object
        m_drivingSparkMax.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure Turning motor
        SparkMaxConfig turningConfig = new SparkMaxConfig();
        turningConfig
            .idleMode(ModuleConstants.kTurningMotorIdleMode)
            .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)
            .inverted(ModuleConstants.kTurningMotorsInverted);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        turningConfig.encoder
            .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

        // Sets PID to use relative encoder
        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
            .positionWrappingEnabled(true) 
            .positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput)
            .positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput)

        // Set the PID gains for the turning motor.
            .p(ModuleConstants.kTurningP)
            .i(ModuleConstants.kTurningI)
            .d(ModuleConstants.kTurningD)
            .velocityFF(ModuleConstants.kTurningFF)
            .outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

        // Configure turning Spark Max with configuration object
        m_turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningEncoder = m_turningSparkMax.getEncoder();
        m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
        m_turningPIDController = m_turningSparkMax.getClosedLoopController();

        // m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingEncoder.setPosition(0);
    }

    /**
     * Get the angle that the swerve wheel is facing.
     * @return The angle as a <code>Rotation2d</code> object.
     * @see Rotation2d
     */
    public Rotation2d getTurningAngle() {
        
        double currentEncoderValue = Units.rotationsToRadians(
            m_turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()
        );
        
        // Send the absolute value into the relative encoder if the value is new.
        // If the encoder is not working, the value will default to 0.
        if ((currentEncoderValue != m_mostRecentTurningEncoderValue) && (currentEncoderValue != 0)) {
            m_turningEncoder.setPosition(currentEncoderValue);
            m_mostRecentTurningEncoderValue = currentEncoderValue;
            return Rotation2d.fromRadians(currentEncoderValue);
        }

        return Rotation2d.fromRadians(m_turningEncoder.getPosition());
    }

    /**
     * Get the angle of the swerve module reported by the built-in NEO encoders.
     * @return The angle in degrees.
     */
    public double getRelativeTurningAngle() {
        return Units.radiansToDegrees(m_turningSparkMax.getEncoder().getPosition());
    }

    /**
     * Get the angle the swerve module is trying to be at.
     * @return The angle in degrees.
     */
    public double getDesiredAngle() {
        return Units.radiansToDegrees(m_desiredAngle);
    }

    /**
     * Gets the current state of the module.
     * @return The current state of the module as a <code>SwerveModuleState</code>.
     * @see SwerveModuleState
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(m_drivingEncoder.getVelocity(), getTurningAngle());
    }

    /**
     * Returns the current position of the module.
     * @return The current position of the module as a <code>SwerveModulePosition</code>.
     * @see SwerveModulePosition
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(m_drivingEncoder.getPosition(), getTurningAngle());
    }

    /**
     * Sets the desired state for the module.
     * @param desiredState Desired state with speed and angle as a <code>SwerveModuleState</code>
     * @see SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle;

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(getTurningAngle());

        // Do cosine scaling on the drive speed to reduce perpendicular movement. 
        // correctedDesiredState.cosineScale(getTurningAngle());

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        m_drivingPIDController.setReference(correctedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
        m_turningPIDController.setReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

        m_desiredAngle = correctedDesiredState.angle.getRadians();
    }

    /** Zeroes all the <code>SwerveModule</code> encoders. */
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }

    /**
     * Sets the voltage of the drive motor.
     * @param volts The voltage to set as a <code>Voltage</code> object.
     * @see Voltage
     */
    public void setDriveVoltage(Voltage volts) {
        m_drivingPIDController.setReference(volts.in(Volts), ControlType.kVoltage);
    }

    /**
     * Sets the voltage of the turning motor.
     * @param volts The voltage to set as a <code>Voltage</code> object.
     * @see Voltage
     */
    public void setTurningVoltage(Voltage volts) {
        m_drivingPIDController.setReference(volts.in(Volts), ControlType.kVoltage);
    }

    /**
     * Sets the voltage of the drive motor.
     * @param volts The voltage to set.
     */
    public void setDriveVoltage(double volts) {
        m_drivingPIDController.setReference(volts, ControlType.kVoltage);
    }

    /**
     * Sets the voltage of the turning motor.
     * @param volts The voltage to set.
     */
    public void setTurningVoltage(double volts) {
        m_drivingPIDController.setReference(volts, ControlType.kVoltage);
    }
}
