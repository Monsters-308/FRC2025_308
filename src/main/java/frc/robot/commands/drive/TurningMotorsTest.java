package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class TurningMotorsTest extends Command {

    private final DriveSubsystem m_driveSubsystem;
    private double m_moduleAngle = 0;

    /** 
     * Makes all of the wheels on the drivetrain spin around.
     * Useful for testing if any of the turning motors are broken.
     * @param subsystem
     */
    public TurningMotorsTest(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        
        addRequirements(driveSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveSubsystem.setModuleStates(new SwerveModuleState[]{
            new SwerveModuleState(0, Rotation2d.fromDegrees(m_moduleAngle)), 
            new SwerveModuleState(0, Rotation2d.fromDegrees(m_moduleAngle)), 
            new SwerveModuleState(0, Rotation2d.fromDegrees(m_moduleAngle)), 
            new SwerveModuleState(0, Rotation2d.fromDegrees(m_moduleAngle))
        });

        // Change angle by small amount
        m_moduleAngle += 2;
    }
}

