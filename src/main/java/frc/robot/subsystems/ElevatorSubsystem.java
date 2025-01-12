package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
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

    /** Encoder that repreents the elevator motors. */
    private final RelativeEncoder m_elevatorEncoder;

    /** PID controller the controls the elevator motors. */
    private final SparkClosedLoopController m_elevatorPIDController;

    /**
     * Initializes an ElevatorSubsystem to control the robot's subsystem.
     */
    public ElevatorSubsystem() {
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();

        leftMotorConfig.idleMode(ElevatorConstants.kElevatorIdleMode);
        leftMotorConfig.smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit);

        leftMotorConfig.inverted(ElevatorConstants.kElevatorLeftInverted);

        leftMotorConfig.encoder.positionConversionFactor(ElevatorConstants.kElevatorEncoderPositionFactor);
        leftMotorConfig.encoder.velocityConversionFactor(ElevatorConstants.kElevatorEncoderVelocityFactor);

        leftMotorConfig.closedLoop.p(ElevatorConstants.kElevatorP);
        leftMotorConfig.closedLoop.i(ElevatorConstants.kElevatorI);
        leftMotorConfig.closedLoop.d(ElevatorConstants.kElevatorD);
        leftMotorConfig.closedLoop.velocityFF(ElevatorConstants.kElevatorFF);
        leftMotorConfig.closedLoop.outputRange(ElevatorConstants.kElevatorMinOutput, 
            ElevatorConstants.kElevatorMaxOutput);

        m_elevatorLeft.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

        rightMotorConfig.idleMode(ElevatorConstants.kElevatorIdleMode);
        rightMotorConfig.smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit);

        rightMotorConfig.inverted(ElevatorConstants.kElevatorRightInverted);

        rightMotorConfig.follow(m_elevatorLeft);

        m_elevatorRight.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_elevatorEncoder = m_elevatorLeft.getEncoder();
        m_elevatorPIDController = m_elevatorLeft.getClosedLoopController();

        goDown();
    }

    /**
     * Starts moving the elevator up.
     */
    public void goUp() {
        m_elevatorPIDController.setReference(ElevatorConstants.kElevatorSpeedMetersPerSecond, ControlType.kVelocity);
    }

    /**
     * Starts moving the elevator down.
     */
    public void goDown() {
        m_elevatorPIDController.setReference(-ElevatorConstants.kElevatorSpeedMetersPerSecond, ControlType.kVelocity);
    }

    /**
     * Moves the elevators to the specified level.
     * @param index The index of the level.
     */
    public void goToLevel(int index) {
        m_elevatorPIDController.setReference(ElevatorConstants.kElevatorLevelHeights[index], ControlType.kPosition);
    }

    /**
     * Creates a Command that moves to elevator to the specified level.
     * @param index The index of the level.
     * @return The runnable Command.
     */
    public Command runGoToLevel(int index) {
        return runOnce(() -> goToLevel(index));
    }

    /** 
     * Gets the current closest level of the elevator.
     * @return The index of the level.
     */
    public int getCurrentLevel() {
        double height = getElevatorHeight();
        double smallestDifference = Integer.MAX_VALUE;
        int index = 0;

        for (int i = 0; i < ElevatorConstants.kElevatorLevelHeights.length; i++) {
            double difference = Math.abs(ElevatorConstants.kElevatorLevelHeights[i] - height);

            if (difference < smallestDifference) {
                smallestDifference = difference;
                index = i;
            }
        }

        return index;
    }

    /**
     * Gets the current height of the elevator in meters 
     */
    public double getElevatorHeight() {
        return m_elevatorEncoder.getPosition();
    }

    /**
     * Stops the elevator.
     */
    public void stop() {
        m_elevatorLeft.set(0);
    }
}
