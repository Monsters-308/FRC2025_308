package frc.robot.subsystems;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConstants;

/**
 * Subsystem that interfaces with Photon Vision for april tag position estimation.
 */
public class PhotonSubsystem extends SubsystemBase {
    /** The {@link PhotonCamera} that represents the robot camera. */
    private PhotonCamera m_camera = new PhotonCamera(PhotonConstants.kCameraName);
    private PhotonPoseEstimator m_photonPoseEstimator = new PhotonPoseEstimator(
            PhotonConstants.kFieldLayout, 
            PhotonConstants.kPoseStrategy, 
            PhotonConstants.kRobotToCamera
        );

    /**
     * Gets the current position estimation from Photon Vision.
     * @return An optional {@link EstimatedRobotPose} that contains an estimated {@link Pose2d} and timestamp if present.
     */
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

        if (latestResult == null) {
            return Optional.empty();
        }

        return m_photonPoseEstimator.update(latestResult);
    }

    /**
     * Sets the driver mode of the camera.
     * @param mode Whether to turn the driver mode on or off.
     */
    public void setDriverMode(boolean mode) {
        m_camera.setDriverMode(mode);
    }

    /**
     * Sets the pipeline of the camera.
     * @param mode The pipeline index to set the camera to.
     */
    public void setPipeline(int index) {
        m_camera.setPipelineIndex(index);
    }
}