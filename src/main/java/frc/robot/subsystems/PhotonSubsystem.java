package frc.robot.subsystems;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
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
        List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
        Double latestTimestamp = null;

        PhotonPipelineResult latestResult = null;

        for (int i = 0; i < results.size(); i++) {
            double timestamp = results.get(i).getTimestampSeconds();
            if (timestamp > latestTimestamp || latestTimestamp == null) {
                latestResult = results.get(i);
                latestTimestamp = timestamp;
            }
        }

        return m_photonPoseEstimator.update(latestResult);
    }

    public void setDriverMode(boolean mode) {
        m_camera.setDriverMode(mode);
    }

    public void setPipeline(int index) {
        m_camera.setPipelineIndex(index);
    }
}