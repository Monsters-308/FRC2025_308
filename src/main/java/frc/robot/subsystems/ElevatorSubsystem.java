package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
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
        MotorType.kBrushless);

    /** The right motor controller of the elevator. */
    private final SparkMax m_elevatorRight = new SparkMax(
        ElevatorConstants.kElevatorRightCanId,
        MotorType.kBrushless);

    /** Encoder that represents the elevator motors. */
    private final RelativeEncoder m_elevatorEncoder;

    /** PID controller the controls the elevator motors. */
    private final SparkClosedLoopController m_elevatorPIDController;

    /** Elevator bottom limit switch. */
    private final DigitalInput m_bottomSwitch = new DigitalInput(ElevatorConstants.kBottomSwitchChannel);

    /** Elevator top limit switch. */
    private final DigitalInput m_topSwitch = new DigitalInput(ElevatorConstants.kTopSwitchChannel);

    /** The maximum height the elevator is able to go to. */
    private Double m_maxElevatorHeight;

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
        leftMotorConfig.closedLoop.outputRange(
            -ElevatorConstants.kElevatorMaxMetersPerSecond / ElevatorConstants.kElevatorFreeSpeedMetersPerSecond, 
            ElevatorConstants.kElevatorMaxMetersPerSecond / ElevatorConstants.kElevatorFreeSpeedMetersPerSecond);

        m_elevatorLeft.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

        rightMotorConfig.idleMode(ElevatorConstants.kElevatorIdleMode);
        rightMotorConfig.smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit);

        rightMotorConfig.inverted(ElevatorConstants.kElevatorRightInverted);

        rightMotorConfig.follow(m_elevatorLeft);

        m_elevatorRight.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_elevatorEncoder = m_elevatorLeft.getEncoder();
        m_elevatorPIDController = m_elevatorLeft.getClosedLoopController();
    }

    /**
     * Sets the current speed of the elevator.
     * @param speed The speed to set the elevator to in meters per second.
     */
    public void setElevatorSpeed(double speed) {
        m_elevatorPIDController.setReference(speed, ControlType.kVelocity);
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
     * Moves the elevator to the specified height.
     * @param height The height to move the elevator to in meters.
     */
    public void setElevatorHeight(double height) {
        m_elevatorPIDController.setReference(height, ControlType.kPosition);
    }

    /**
     * Sets the maximum height the elevator can go to after calibration.
     * @param maxHeight The maximum height to set.
     */
    public void setMaxHeight(double maxHeight) {
        m_maxElevatorHeight = maxHeight;
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
     * Gets the current height of the elevator in meters.
     * @return The height of the elevator.
     */
    public double getElevatorHeight() {
        return m_elevatorEncoder.getPosition();
    }

    /**
     * Gets the current speed of the elevator.
     * @return The speed of the elevator in meters per second.
     */
    public double getElevatorSpeed() {
        return m_elevatorEncoder.getVelocity();
    }

    /**
     * Stops the elevator.
     */
    public void stop() {
        m_elevatorLeft.set(0);
    }

    @Override
    public void periodic() {
        if (m_bottomSwitch.get()) {
            stop();
            m_elevatorEncoder.setPosition(0);
        } else if (m_topSwitch.get()) {
            if (m_maxElevatorHeight != null) {
                m_elevatorEncoder.setPosition(m_maxElevatorHeight);
            }
            stop();
        }
    }
}
