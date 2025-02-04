package frc.robot.subsystems;
import java.util.Optional;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConstants;


public class PhotonSubsystem extends SubsystemBase {

    private PhotonCamera m_camera = new PhotonCamera(PhotonConstants.kCameraName);
    private PhotonPoseEstimator m_photonPoseEstimator = new PhotonPoseEstimator(
            PhotonConstants.kFeildLayout, 
            PhotonConstants.kPoseStrategy, 
            PhotonConstants.kRobotToCamera
        );

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return m_photonPoseEstimator.update(m_camera.getLatestResult());
    }

    public void setDriverMode(boolean mode) {
        m_camera.setDriverMode(mode);
    }

    public void setPipeline(int index) {
        m_camera.setPipelineIndex(index);
    }
}