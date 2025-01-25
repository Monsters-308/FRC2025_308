package frc.robot.subsystems;

import java.util.Map;
import java.util.function.IntSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
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
    private final ProfiledPIDController m_elevatorPIDController = new ProfiledPIDController(
        ElevatorConstants.kElevatorP,
        ElevatorConstants.kElevatorI,
        ElevatorConstants.kElevatorD,
        new TrapezoidProfile.Constraints(
            ElevatorConstants.kElevatorMaxSpeedMetersPerSecond,
            ElevatorConstants.kElevatorMaxAccelerationMetersPerSecondSquared
        )
    );

    /** Elevator bottom limit switch. */
    private final DigitalInput m_bottomSwitch = new DigitalInput(ElevatorConstants.kBottomSwitchChannel);

    /** Elevator top limit switch. */
    private final DigitalInput m_topSwitch = new DigitalInput(ElevatorConstants.kTopSwitchChannel);

    /** A shuffleboard tab to write elevator properties to the dashboard. */
    private final ShuffleboardTab m_elevatorTab = Shuffleboard.getTab("Elevator");
    /** A shuffleboard layout that holds the go to level command. */
    private final ShuffleboardLayout m_goToLevelLayout = m_elevatorTab.getLayout("Go To Level");
    /** The network table entry that contains the level to send the robot to when the dashboard button is pressed. */
    private final GenericEntry m_levelNetworkTableEntry;

    /** Whether or not the elevator is currently being calibrated. */
    private boolean m_isPIDMode = false;

    /**
     * Initializes an ElevatorSubsystem to control the robot's elevator.
     */
    public ElevatorSubsystem() {
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();

        leftMotorConfig
            .idleMode(ElevatorConstants.kElevatorIdleMode)
            .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
            .inverted(ElevatorConstants.kElevatorLeftInverted);

        leftMotorConfig.encoder
            .positionConversionFactor(ElevatorConstants.kElevatorEncoderPositionFactor)
            .velocityConversionFactor(ElevatorConstants.kElevatorEncoderVelocityFactor);

        m_elevatorLeft.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

        rightMotorConfig
            .idleMode(ElevatorConstants.kElevatorIdleMode)
            .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
            .inverted(ElevatorConstants.kElevatorRightInverted)
            .follow(m_elevatorLeft);

        m_elevatorRight.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_elevatorEncoder = m_elevatorLeft.getEncoder();
        
        Preferences.initDouble(ElevatorConstants.kPhysicalHeightLimitKey, ElevatorConstants.kPhysicalHeightLimit);

        m_elevatorTab.addDouble("Current Elevator Height", this::getElevatorHeight);
        m_elevatorTab.addInteger("Elevator Level", this::getCurrentLevel);
        m_elevatorTab.addDouble("Elevator Speed", this::getElevatorVelocity);

        m_elevatorTab.add("Maximum Elevator Speed", ElevatorConstants.kElevatorMaxSpeedMetersPerSecond);
        m_elevatorTab.addDouble("Physical Height Limit", this::getPhysicalHeightLimit);

        m_elevatorTab.add("Calibrate Elevator", calibrateElevator());

        m_levelNetworkTableEntry = m_goToLevelLayout.add("Level", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("Min", 1,
                "Max", ElevatorConstants.kElevatorLevelHeights.length,
                "Block Increment", 1))
            .getEntry();

        m_goToLevelLayout.add(goToLevel(() -> (int)m_levelNetworkTableEntry.getInteger(-1)));
    }

    /**
     * Resets and set the goal for the PID controller.
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
     * Creates a Command that moves to elevator to the specified level.
     * @param index The index of the level.
     * @return The runnable Command.
     */
    public Command goToLevel(int index) {
        return runOnce(() -> setElevatorHeight(ElevatorConstants.kElevatorLevelHeights[index]))
            .andThen(new WaitUntilCommand(m_elevatorPIDController::atGoal));
    }

    /**
     * Creates a Command that moves to elevator to the level returned from the index supplier.
     * @param indexSupplier The supplier of the index of the level.
     * @return The runnable Command.
     */
    public Command goToLevel(IntSupplier indexSupplier) {
        return runOnce(() -> {
            int index = indexSupplier.getAsInt();
            if (index < 0) { return; }
            setElevatorHeight(ElevatorConstants.kElevatorLevelHeights[index]);
        })
        .andThen(new WaitUntilCommand(m_elevatorPIDController::atGoal));
    }

    /**
     * Creates a command that moves the elevator to the specified height.
     * @param height The height to move the elevator to in meters.
     * @return The runnable Command.
     */
    public Command goToHeight(double height) {
        return runOnce(() -> setElevatorHeight(height))
            .andThen(new WaitUntilCommand(m_elevatorPIDController::atGoal));
    }

    /**
     * Sets the physical height limit the elevator can reach. Used for calibration.
     * @param height The new physical height limit to set.
     */
    public void setPhysicalHeightLimit(double height) {
        Preferences.setDouble(ElevatorConstants.kPhysicalHeightLimitKey, height);
    }

    /**
     * Sets the velocity of the elevator.
     * @param velocity The velocity to set in meters per second.
     */
    public void setElevatorVelocity(double velocity) {
        m_isPIDMode = false;
        m_elevatorLeft.set(velocity / ElevatorConstants.kElevatorFreeSpeedMetersPerSecond);
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
     * Gets the physical height limit the elevator can reach.
     * @return The physical height limit of the elevator.
     */
    public double getPhysicalHeightLimit() {
        return Preferences.getDouble(ElevatorConstants.kPhysicalHeightLimitKey, ElevatorConstants.kPhysicalHeightLimit);
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
     * Creates a command that calibrates the elevator.
     * @return The runnable Command.
     */
    public Command calibrateElevator() {
        return runOnce(() -> {
            setElevatorVelocity(-ElevatorConstants.kElevatorMaxSpeedMetersPerSecond);
        })
        .andThen(new WaitUntilCommand(() -> getElevatorVelocity() == 0))
        .andThen(() -> m_elevatorLeft.set(ElevatorConstants.kElevatorMaxSpeedMetersPerSecond))
        .andThen(new WaitUntilCommand(() -> getElevatorVelocity() == 0))
        .finallyDo(() -> {
            setPhysicalHeightLimit(getElevatorHeight());
            setElevatorHeight(0);
        });
    }

    @Override
    public void periodic() {
        if (m_isPIDMode) {
            m_elevatorLeft.set(
                m_elevatorPIDController.calculate(getElevatorHeight()) + 
                ElevatorConstants.kElevatorGravityOffset
            );
        }

        // Prevent elevator motors from moving after the elevator cannot move any further
        if (m_bottomSwitch.get()) {
            stop();
            m_elevatorEncoder.setPosition(0); // Reset encoder position to 0 at the bottom
            m_elevatorPIDController.reset(0, 0);
        } else if (m_topSwitch.get()) {
            stop();
            m_elevatorEncoder.setPosition(getPhysicalHeightLimit()); // Reset encoder position to the height of the elevator at the top
            m_elevatorPIDController.reset(getPhysicalHeightLimit(), 0);
        }

        // Prevent level in shuffleboard go to level layout from being non-integer
        double value = m_levelNetworkTableEntry.getDouble(-1);
        long roundedValue = Math.round(value);
        if (roundedValue != value) {
            m_levelNetworkTableEntry.setInteger(roundedValue);
        }
    }
}
