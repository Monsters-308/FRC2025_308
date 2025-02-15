package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.utils.Utils;
import frc.utils.ThroughBoreEncoder;

/**
 * Subsystem that controls the coral arm of the robot.
 */
public class ArmSubsystem extends SubsystemBase {
    /** The motor controller for the coral arm. */
    private final SparkMax m_armMotor = new SparkMax(ArmConstants.kArmMotorCanId, MotorType.kBrushless);
    /** The encoder for measuring the position and velocity of the motor. */
    private final ThroughBoreEncoder m_armEncoder = new ThroughBoreEncoder(
        ArmConstants.kArmEncoderId,
        ArmConstants.kEncoderInverted,
        ArmConstants.kEncoderAngleOffset,
        ArmConstants.kArmDutyCyclePeriod
    );

    /** The {@link ProfiledPIDController} for the arm motor. */
    private final ProfiledPIDController m_angleController = new ProfiledPIDController(
        ArmConstants.KArmP,
        ArmConstants.KArmI,
        ArmConstants.KArmD,
        new TrapezoidProfile.Constraints(
            ArmConstants.kArmMaxSpeedRPS,
            ArmConstants.kArmMaxAccelerationRPSSquared
        )
    );

    /** The {@link ArmFeedforward} for the arm motor. */
    private final ArmFeedforward m_armFeedforward = new ArmFeedforward(
        ArmConstants.kArmS,
        ArmConstants.kArmG,
        ArmConstants.kArmV,
        ArmConstants.kArmA
    );

    /** Whether to use PID or not. */
    private boolean m_isPIDMode = true;

    /** A {@link SuffleboardTab} to write arm properties to the dashboard. */
    private final ShuffleboardTab m_armTab = Shuffleboard.getTab("Arm");

    /**
     * Constructs an {@link ArmSubsystem} that controls the coral arm of the robot.
     */
    public ArmSubsystem() {
        SparkMaxConfig armMotorConf = new SparkMaxConfig();
        armMotorConf
            .inverted(ArmConstants.kArmMotorInvered)
            .smartCurrentLimit(ArmConstants.kSmartCurrentLimit)
            .idleMode(ArmConstants.kIdleMode);

        m_armMotor.configure(armMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_armTab.addDouble("Arm Angle", () -> getAngle().getDegrees());
        m_armTab.addDouble("Arm Velocity", () -> getVelocity().getDegrees());

        m_armTab.addDouble("Arm Angle Setpoint", () -> 
            Units.rotationsToDegrees(m_angleController.getSetpoint().position));
        m_armTab.addDouble("Arm Velocity Setpoint", () -> 
            Units.rotationsToDegrees(m_angleController.getSetpoint().velocity));

        m_armTab.addDouble("Arm Angle Goal", () -> 
            Units.rotationsToDegrees(m_angleController.getGoal().position));
        m_armTab.addDouble("Arm Velocity Goal", () -> 
            Units.rotationsToDegrees(m_angleController.getGoal().velocity));

        Utils.configureSysID(
            m_armTab.getLayout("Arm SysID", BuiltInLayouts.kList), this, 
            voltage -> {
                m_isPIDMode = false;
                m_armMotor.setVoltage(voltage);
            }
        );
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
     * Gets the current angle of the coral arm.
     * @return The current angle of the arm.
     */
    public Rotation2d getAngle() {
        return m_armEncoder.getRotation2D();
    }

    /**
     * Gets the current velocity of the arm.
     * @return The velocity of the arm as a {@link Rotation2d} object.
     */
    public Rotation2d getVelocity() {
        return m_armEncoder.getRate();
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
            State setpoint = m_angleController.getSetpoint();
            double velocitySetpoint = Units.rotationsToRadians(setpoint.velocity);

            m_armMotor.setVoltage(
                m_angleController.calculate(m_armEncoder.getRotations()) +
                m_armFeedforward.calculateWithVelocities(getAngle().getRadians(), getVelocity().getRadians(), velocitySetpoint)
            );
        }
    }
}