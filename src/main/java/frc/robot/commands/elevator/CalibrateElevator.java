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
     * Creates a CalibrateElevator Command that calibrates the elevator encoders.
     * @param elevatorSubsystem The elevator subsystem that represents the elevator.
     */
    public CalibrateElevator(ElevatorSubsystem elevatorSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;

        addRequirements(m_elevatorSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_elevatorSubsystem.setElevatorSpeed(-ElevatorConstants.kElevatorMaxMetersPerSecond);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_elevatorSubsystem.getElevatorSpeed() == 0) {
            if (m_hasReachedBottom) {
                m_hasReachedBottom = true;
                m_elevatorSubsystem.setElevatorSpeed(ElevatorConstants.kElevatorMaxMetersPerSecond);
            } else {
                m_isFinished = true;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.setElevatorSpeed(-ElevatorConstants.kElevatorMaxMetersPerSecond);
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
