package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Command that calibrates the elevator encoders.
 * @param elevatorSubsystem The ElevatorSubsystem that represents the elevator.
 */
public class CalibrateElevator extends Command {
    /** The ElevatorSubsystem that represents the elevator. */
    private final ElevatorSubsystem m_elevatorSubsystem;

    /** Whether the elevator has gotten to its lowest point after the start of calibration. */
    private boolean m_hasReachedBottom = false;
    /** Whether the calibration is finished. */
    private boolean m_isFinished = false;

    /**
     * Creates a CalibrateElevator command object that calibrates the elevator encoders.
     * @param elevatorSubsystem The elevator subsystem that represents the elevator.
     */
    public CalibrateElevator(ElevatorSubsystem elevatorSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;

        addRequirements(m_elevatorSubsystem);
    }
    
    @Override
    public void initialize() {
        // Moves elevator towards bottom
        m_elevatorSubsystem.setElevatorSpeed(-ElevatorConstants.kElevatorMaxMetersPerSecond);
    }

    @Override
    public void execute() {
        if (m_elevatorSubsystem.getElevatorSpeed() == 0) {
            if (m_hasReachedBottom) {
                // Once it reaches the bottom, moves the elevator to the top
                m_hasReachedBottom = true;
                m_elevatorSubsystem.setElevatorSpeed(ElevatorConstants.kElevatorMaxMetersPerSecond);
            } else {
                // Sets elevator max height once it reaches the top, then finishes the command
                m_elevatorSubsystem.setMaxHeight(m_elevatorSubsystem.getElevatorHeight());
                m_isFinished = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Moves elevator back towards the bottom
        m_elevatorSubsystem.setElevatorSpeed(-ElevatorConstants.kElevatorMaxMetersPerSecond);
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
