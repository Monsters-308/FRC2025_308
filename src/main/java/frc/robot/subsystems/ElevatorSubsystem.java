package frc.robot.subsystems;

import java.util.Map;
import java.util.function.IntSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.elevator.CalibrateElevator;

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

    /** A shuffleboard tab to write elevator properties to the dashboard. */
    private final ShuffleboardTab m_elevatorTab = Shuffleboard.getTab("Elevator");
    /** A shuffleboard layout that holds the go to level command. */
    private final ShuffleboardLayout m_goToLevelLayout = m_elevatorTab.getLayout("Go To Level");
    /** The network table entry that contains the level to send the robot to when the dashboard button is pressed. */
    private final GenericEntry m_levelNetworkTableEntry;

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
        
        Preferences.initDouble(ElevatorConstants.kPhysicalHeightLimitKey, ElevatorConstants.kPhysicalHeightLimit);

        m_elevatorTab.addDouble("Current Elevator Height", this::getElevatorHeight);
        m_elevatorTab.addInteger("Elevator Level", this::getCurrentLevel);
        m_elevatorTab.addDouble("Elevator Speed", this::getElevatorSpeed);

        m_elevatorTab.add("Maximum Elevator Speed", ElevatorConstants.kElevatorMaxMetersPerSecond);
        m_elevatorTab.addDouble("Physical Height Limit", this::getPhysicalHeightLimit);

        m_elevatorTab.add("Calibrate Elevator", new CalibrateElevator(this));

        m_levelNetworkTableEntry = m_goToLevelLayout.add("Level", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("Min", 1,
                "Max", ElevatorConstants.kElevatorLevelHeights.length,
                "Block Increment", 1))
            .getEntry();

        m_goToLevelLayout.add(runGoToLevel(() -> (int)m_levelNetworkTableEntry.getInteger(-1)));
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
     * Creates a Command that moves to elevator to the level returned from the index supplier.
     * @param indexSupplier The supplier of the index of the level.
     * @return The runnable Command.
     */
    public Command runGoToLevel(IntSupplier indexSupplier) {
        return runOnce(() -> {
            int index = indexSupplier.getAsInt();
            if (index < 0) { return; }
            goToLevel(index);
        });
    }

    /**
     * Moves the elevator to the specified height.
     * @param height The height to move the elevator to in meters.
     */
    public void goToHeight(double height) {
        m_elevatorPIDController.setReference(height, ControlType.kPosition);
    }

    /**
     * Sets the physical height limit the elevator can reach. Used for calibration.
     * @param height The new physical height limit to set.
     */
    public void setPhysicalHeightLimit(double height) {
        Preferences.setDouble(ElevatorConstants.kPhysicalHeightLimitKey, height);
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
     * Gets the physical height limit the elevator can reach.
     * @return The physical height limit of the elevator.
     */
    public double getPhysicalHeightLimit() {
        return Preferences.getDouble(ElevatorConstants.kPhysicalHeightLimitKey, ElevatorConstants.kPhysicalHeightLimit);
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
        // Prevent elevator motors from moving after the elevator cannot move any further
        if (m_bottomSwitch.get()) {
            stop();
            m_elevatorEncoder.setPosition(0); // Reset encoder position to 0 at the bottom
        } else if (m_topSwitch.get()) {
            m_elevatorEncoder.setPosition(getPhysicalHeightLimit()); // Reset encoder position to the height of the elevator at the top
            stop();
        }

        // Prevent level in shuffleboard go to level layout from being non-integer
        double value = m_levelNetworkTableEntry.getDouble(-1);
        long roundedValue = Math.round(value);
        if (roundedValue != value) {
            m_levelNetworkTableEntry.setInteger(roundedValue);
        }
    }
}
