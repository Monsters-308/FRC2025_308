package frc.robot.commands.vision;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class UpdateOdometry extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final VisionSubsystem m_visionSubsystem;

    /**
     * Uses limelight data to reset robot pose.
     */
    public UpdateOdometry(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pose2d robotPose = m_visionSubsystem.getRobotPosition();
        if(robotPose != null){
            m_driveSubsystem.resetOdometry(robotPose);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}