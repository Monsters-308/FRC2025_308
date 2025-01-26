package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.utils.SwerveUtils;
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

    /** The PID controller for the arm motor. */
    private final ProfiledPIDController m_angleController = new ProfiledPIDController(
        ArmConstants.KArmP,
        ArmConstants.KArmI,
        ArmConstants.KArmD,
        new TrapezoidProfile.Constraints(
            ArmConstants.kArmMaxSpeedRPS,
            ArmConstants.kArmMaxAccelerationRPSSquared
        )
    );

    /**
     * Constructs an arm subsystem that controls the coral arm of the robot.
     */
    public ArmSubsystem() {
        SparkMaxConfig armMotorConf = new SparkMaxConfig();
        armMotorConf
            .inverted(ArmConstants.kArmMotorInvered)
            .smartCurrentLimit(ArmConstants.kSmartCurrentLimit)
            .idleMode(ArmConstants.kIdleMode);

        m_armMotor.configure(armMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Resets and sets the goal of the angle PID controller.
     * @param angle The angle to set.
     */
    public void setAngle(Rotation2d angle) {
        Rotation2d constrainedAngle = Rotation2d.fromDegrees(SwerveUtils.angleConstrain(angle.getDegrees()));
        m_angleController.reset(
            getAngle().getRotations()
        );

        m_angleController.setGoal(constrainedAngle.getRotations());
    }

    /**
     * Creates a command the moves the arm to the specified angle.
     * @param angle The angle the arm should move to.
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
     * Stops movement of the coral arm.
     */
    public void stop() {
        setAngle(getAngle());
    }

    @Override
    public void periodic() {
        double gravityOffset = m_armEncoder.getRotation2D().getSin() * ArmConstants.kGravityOffsetMultiplier;
        m_armMotor.set(m_angleController.calculate(m_armEncoder.getRotations()) + gravityOffset);
    }
}