package frc.robot.subsystems;
import java.util.Optional;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConstants;


public class PhotonSubsystem extends SubsystemBase {

    private PhotonCamera camera = new PhotonCamera(PhotonConstants.kCameraName);
    private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
            PhotonConstants.kFeildLayout, 
            PhotonConstants.kPoseStrategy, 
            PhotonConstants.kRobotToCamera
        );

    public PhotonSubsystem() {

    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

}