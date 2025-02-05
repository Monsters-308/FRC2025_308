package frc.robot.subsystems;

import java.util.Map;
import java.util.function.IntSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorConstants;

/**
 * Subsystem that controls the robot's elevator.
 */
public class ElevatorSubsystem extends SubsystemBase {
    /** The left motor controller of the elevator. */
    private final SparkMax m_elevatorLeader = new SparkMax(
        ElevatorConstants.kElevatorLeftCanId,
        MotorType.kBrushless);

    /** The right motor controller of the elevator. */
    private final SparkMax m_elevatorFollower = new SparkMax(
        ElevatorConstants.kElevatorRightCanId,
        MotorType.kBrushless);

    /** Encoder that represents the elevator motors. */
    private final RelativeEncoder m_elevatorEncoder;

    /** PID controller the controls the elevator motors. */
    private final ProfiledPIDController m_elevatorPIDController = new ProfiledPIDController(
        ElevatorConstants.kElevatorP,
        ElevatorConstants.kElevatorI,
        ElevatorConstants.kElevatorD,
        new TrapezoidProfile.Constraints(
            ElevatorConstants.kElevatorMaxSpeedMetersPerSecond,
            ElevatorConstants.kElevatorMaxAccelerationMetersPerSecondSquared
        )
    );

    private final ElevatorFeedforward m_elevatorFeedforward = new ElevatorFeedforward(
        ElevatorConstants.kElevatorS,
        ElevatorConstants.kElevatorG,
        ElevatorConstants.kElevatorV,
        ElevatorConstants.kElevatorA
    );

    /** Elevator bottom limit switch. */
    private final DigitalInput m_bottomSwitch = new DigitalInput(ElevatorConstants.kBottomSwitchChannel);

    /** A shuffleboard tab to write elevator properties to the dashboard. */
    private final ShuffleboardTab m_elevatorTab = Shuffleboard.getTab("Elevator");
    /** A shuffleboard layout that holds the go to level command. */
    private final ShuffleboardLayout m_goToLevelLayout = m_elevatorTab.getLayout("Go To Level");
    /** The network table entry that contains the level to send the robot to when the dashboard button is pressed. */
    private final GenericEntry m_levelNetworkTableEntry;

    /** Whether or not the elevator is currently using PID or setting the speed directly. */
    private boolean m_isPIDMode = false;

    /**
     * Initializes an ElevatorSubsystem to control the robot's elevator.
     */
    public ElevatorSubsystem() {
        SparkMaxConfig leaderMotorConfig = new SparkMaxConfig();

        leaderMotorConfig
            .idleMode(ElevatorConstants.kElevatorIdleMode)
            .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
            .inverted(ElevatorConstants.kElevatorLeftInverted);

        leaderMotorConfig.encoder
            .positionConversionFactor(ElevatorConstants.kElevatorEncoderPositionFactor)
            .velocityConversionFactor(ElevatorConstants.kElevatorEncoderVelocityFactor);

        m_elevatorLeader.configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig followerMotorConfig = new SparkMaxConfig();

        followerMotorConfig
            .idleMode(ElevatorConstants.kElevatorIdleMode)
            .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
            .inverted(ElevatorConstants.kElevatorRightInverted)
            .follow(m_elevatorLeader);

        m_elevatorFollower.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_elevatorEncoder = m_elevatorLeader.getEncoder();

        m_elevatorTab.addDouble("Elevator Height", this::getElevatorHeight);
        m_elevatorTab.addInteger("Elevator Level", this::getCurrentLevel);
        m_elevatorTab.addDouble("Elevator Speed", this::getElevatorVelocity);

        m_elevatorTab.add("Zero Elevator", zeroElevator());

        m_levelNetworkTableEntry = m_goToLevelLayout.add("Level", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("Min", 1,
                "Max", ElevatorConstants.kElevatorLevelHeights.length,
                "Block Increment", 1))
            .getEntry();

        m_goToLevelLayout.add(goToLevel(() -> (int)m_levelNetworkTableEntry.getInteger(-1), false));
    }

    /**
     * Resets and sets the height goal of the PID controller.
     * @param height The height to set.
     */
    public void setElevatorHeight(double height) {
        m_isPIDMode = true;

        m_elevatorPIDController.reset(
            m_elevatorEncoder.getPosition(),
            m_elevatorEncoder.getVelocity()
        );

        m_elevatorPIDController.setGoal(height);
    }

    /**
     * Sets the height goal of the PID controller to the specified level.
     * @param index The index of the level to set.
     */
    public void setElevatorLevel(int index) {
        setElevatorHeight(ElevatorConstants.kElevatorLevelHeights[index]);
    }

    /**
     * Creates a Command that moves to elevator to the specified level.
     * @param index The index of the level.
     * @param endImmediately Whether the command should end immediately or wait until the elevator has reached the level.
     * @return The runnable Command.
     */
    public Command goToLevel(int index, boolean endImmediately) {
        return runOnce(() -> setElevatorHeight(ElevatorConstants.kElevatorLevelHeights[index]))
            .andThen(new WaitUntilCommand(() -> m_elevatorPIDController.atGoal() || endImmediately))
            .withName("Go To Level #" + (index + 1));
    }

    /**
     * Creates a Command that moves to elevator to the level returned from the index supplier.
     * @param indexSupplier The supplier of the index of the level.
     * @param endImmediately Whether the command should end immediately or wait until the elevator has reached the level.
     * @return The runnable Command.
     */
    public Command goToLevel(IntSupplier indexSupplier, boolean endImmediately) {
        return runOnce(() -> {
            int index = indexSupplier.getAsInt();
            if (index < 0) { return; }
            setElevatorHeight(ElevatorConstants.kElevatorLevelHeights[index]);
        })
        .andThen(new WaitUntilCommand(()-> m_elevatorPIDController.atGoal() || endImmediately))
        .withName("Go");
    }

    /**
     * Creates a command that moves the elevator to the specified height.
     * @param height The height to move the elevator to in meters.
     * @param endImmediately Whether the command should end immediately or wait until the elevator has reached the height.
     * @return The runnable Command.
     */
    public Command goToHeight(double height, boolean endImmediately) {
        return runOnce(() -> setElevatorHeight(height))
            .andThen(new WaitUntilCommand(()-> m_elevatorPIDController.atGoal() || endImmediately));
    }

    /**
     * Sets the velocity of the elevator.
     * @param velocity The velocity to set in meters per second.
     */
    public void setElevatorVelocity(double velocity) {
        m_isPIDMode = false;
        m_elevatorLeader.set(velocity / ElevatorConstants.kElevatorFreeSpeedMetersPerSecond);
    }

    /** 
     * Gets the current closest level of the elevator.
     * @return The index of the level.
     */
    public int getCurrentLevel() {
        double height = getElevatorHeight();
        Double smallestDifference = null;
        int index = 0;

        for (int i = 0; i < ElevatorConstants.kElevatorLevelHeights.length; i++) {
            double difference = Math.abs(ElevatorConstants.kElevatorLevelHeights[i] - height);

            // Check if current height difference is smaller than the previous one or if there is no previous one
            if (difference < smallestDifference || smallestDifference == null) {
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
    public void stop() {
        setElevatorHeight(getElevatorHeight());
    }

    /**
     * Creates a command that zeros the elevator.
     * @return The runnable Command.
     */
    public Command zeroElevator() {
        return runOnce(() -> {
            setElevatorVelocity(-ElevatorConstants.kElevatorMaxSpeedMetersPerSecond);
        })
        .andThen(new WaitUntilCommand(() -> getElevatorVelocity() == 0))
        .withName("Zero Elevator");
    }

    @Override
    public void periodic() {
        // Prevent elevator motors from moving after the elevator cannot move any further
        if (m_bottomSwitch.get()) {
            stop();
            m_elevatorEncoder.setPosition(0); // Reset encoder position to 0 at the bottom
            m_elevatorPIDController.reset(0, 0);
        }
        
        if (getElevatorHeight() >= ElevatorConstants.kPhysicalHeightLimit) {
            stop();
            m_elevatorPIDController.reset(ElevatorConstants.kPhysicalHeightLimit, 0);
        }

        if (m_isPIDMode) {
            double velocity = m_elevatorPIDController.getSetpoint().velocity;
            
            m_elevatorLeader.setVoltage(
                m_elevatorPIDController.calculate(getElevatorHeight()) + 
                m_elevatorFeedforward.calculate(velocity)
            );
        }

        // Prevent level in shuffleboard go to level layout from being non-integer
        double value = m_levelNetworkTableEntry.getDouble(-1);
        long roundedValue = Math.round(value);
        if (roundedValue != value) {
            m_levelNetworkTableEntry.setInteger(roundedValue);
        }
    }
}
