package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

/**
 * Subsystem that controls the robot's elevator.
 */
public class ElevatorSubsystem extends SubsystemBase {
    /** The left motor controller of the elevator. */
    private final SparkMax m_elevatorLeft = new SparkMax(
        ElevatorConstants.kElevatorLeftCanId,
        MotorType.kBrushed);

    /** The right motor controller of the elevator. */
    private final SparkMax m_elevatorRight = new SparkMax(
        ElevatorConstants.kElevatorRightCanId,
        MotorType.kBrushed);

    /**
     * Initializes an ElevatorSubsystem to control the robot's subsystem.
     */
    public ElevatorSubsystem() {
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();

        leftMotorConfig.idleMode(ElevatorConstants.kElevatorIdleMode);
        leftMotorConfig.smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit);

        leftMotorConfig.inverted(ElevatorConstants.kElevatorLeftInverted);

        m_elevatorLeft.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

        rightMotorConfig.idleMode(ElevatorConstants.kElevatorIdleMode);
        rightMotorConfig.smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit);

        rightMotorConfig.inverted(ElevatorConstants.kElevatorRightInverted);

        rightMotorConfig.follow(m_elevatorLeft);

        m_elevatorRight.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Starts moving the elevator up.
     */
    public void goUp() {
        m_elevatorLeft.set(ElevatorConstants.elevatorSpeedMetersPerSecond);
    }

    /**
     * Starts moving the elevator down.
     */
    public void goDown() {
        m_elevatorLeft.set(-ElevatorConstants.elevatorSpeedMetersPerSecond);
    }

    /**
     * Stops the elevator.
     */
    public void stop() {
        m_elevatorLeft.set(0);
    }
}
