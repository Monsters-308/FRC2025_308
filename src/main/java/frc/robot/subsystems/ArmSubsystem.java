package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.utils.ThroughBoreEncoder;

/**
 * Subsystem that controls the coral arm of the robot.
 */
public class ArmSubsystem extends SubsystemBase {
    /** The motor controller for the coral arm. */
    private final SparkMax m_armMotor = new SparkMax(ArmConstants.kArmMotorCanId, MotorType.kBrushless);
    /** The encoder for measuring the position and velocity of the motor. */
    private final ThroughBoreEncoder m_armEncoder = new ThroughBoreEncoder(ArmConstants.kArmEncoderId, ArmConstants.kEncoderInverted, ArmConstants.kEncoderAngleOffset, ArmConstants.kArmDutyCyclePeriod);

    // private final ProfiledPIDController m_angleController = new Pro

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

    public void periodic() {

    }

    /*
     * Sets the speed of the motor.
     */
    public void setSpeed() {
        m_armMotor.set(ArmConstants.kArmMaxSpeedMetersPerSecond);
    }

    /**
     * This function gets the angle of where the angle of the arm is currently at.
     * @return
     */
    public Rotation2d getAngle() {
        return m_armEncoder.getRotation2D();
    } 

    
    
}

