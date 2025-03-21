package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.Utils;

public class AutoAilgn extends RobotGotoFieldPose {
    public AutoAilgn(DriveSubsystem m_driveSubsystem) {
        super(m_driveSubsystem, Pose2d.kZero);
    }

    @Override
    public void initialize() {
        Pose2d robotPose = m_driveSubsystem.getPose();
        Double smallestDistance = null;

        boolean flipPose = robotPose.getX() > FieldConstants.kFieldWidthMeters / 2;

        for (Pose2d pose : FieldConstants.kAutoAlignPositions) {
            pose = flipPose ? FieldUtils.flip(pose) : pose;
            double dst = Utils.getDistancePosToPos(robotPose.getTranslation(), pose.getTranslation());
            if (smallestDistance == null || dst < smallestDistance) {
                smallestDistance = dst;
                m_desiredRobotPos = pose;
            }
        }

        super.initialize();
    }
}
