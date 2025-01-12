// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final SparkMax m_drivingSparkMax;
    private final SparkMax m_turningSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkClosedLoopController m_drivingPIDController;
    private final SparkClosedLoopController m_turningPIDController;

    private final double m_chassisAngularOffset;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a Swerve Module and configures the driving and turning motor,
     * encoder, and PID controllers. This configuration is specific to the MK4 and 
     * MK4i swerve modules with 2 NEOs, 2 Spark Maxes and an SRX Mag Encoder.
     */
    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, boolean invertDrive) {
        m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

        SparkMaxConfig drivingConfig = new SparkMaxConfig();

        drivingConfig.idleMode(ModuleConstants.kDrivingMotorIdleMode);
        drivingConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);

        drivingConfig.inverted(invertDrive);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        drivingConfig.encoder.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        drivingConfig.encoder.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        // Set the PID gains for the driving motor.
        drivingConfig.closedLoop.p(ModuleConstants.kDrivingP);
        drivingConfig.closedLoop.i(ModuleConstants.kDrivingI);
        drivingConfig.closedLoop.d(ModuleConstants.kDrivingD);
        drivingConfig.closedLoop.velocityFF(ModuleConstants.kDrivingFF);
        drivingConfig.closedLoop.outputRange(ModuleConstants.kDrivingMinOutput,
            ModuleConstants.kDrivingMaxOutput);

        // Configure driving Spark Max with configuration object
        m_drivingSparkMax.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig turningConfig = new SparkMaxConfig();

        turningConfig.idleMode(ModuleConstants.kTurningMotorIdleMode);
        turningConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        turningConfig.inverted(ModuleConstants.kTurningMotorsInverted);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        turningConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        turningConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

        // Sets PID to use absolute encoder
        turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        turningConfig.closedLoop.positionWrappingEnabled(true);
        turningConfig.closedLoop.positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        turningConfig.closedLoop.positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        // Set the PID gains for the turning motor.
        turningConfig.closedLoop.p(ModuleConstants.kTurningP);
        turningConfig.closedLoop.i(ModuleConstants.kTurningI);
        turningConfig.closedLoop.d(ModuleConstants.kTurningD);
        turningConfig.closedLoop.velocityFF(ModuleConstants.kTurningFF);
        turningConfig.closedLoop.outputRange(ModuleConstants.kTurningMinOutput,
            ModuleConstants.kTurningMaxOutput);

        // Configure turning Spark Max with configuration object
        m_turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
        m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
        m_turningPIDController = m_turningSparkMax.getClosedLoopController();

        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingEncoder.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(m_drivingEncoder.getVelocity(),
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
            m_drivingEncoder.getPosition(),
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        m_drivingPIDController.setReference(correctedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
        m_turningPIDController.setReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

        m_desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }
}
