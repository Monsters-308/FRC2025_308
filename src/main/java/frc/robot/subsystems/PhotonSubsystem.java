package frc.robot.subsystems;
import java.util.ArrayList;
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

    private List<PhotonPipelineResult> m_results;

    /**
     * Gets the current position estimation from Photon Vision.
     * @return An optional {@link EstimatedRobotPose} that contains an estimated {@link Pose2d} and timestamp if present.
     */
    public List<EstimatedRobotPose> getEstimatedGlobalPose() {
        m_results = m_camera.getAllUnreadResults();
        ArrayList<EstimatedRobotPose> poses = new ArrayList<>();

        for (int i = 0; i < m_results.size(); i++) {
            Optional<EstimatedRobotPose> estimationOpt = m_photonPoseEstimator.update(m_results.get(i));
            if (estimationOpt.isPresent()) {
                poses.add(estimationOpt.get());
            } else {
                m_results.remove(i);
            }
        }

        return poses;
    }

    /**
     * Gets the {@link PhotonPipelineResult} at the specified index.
     * @param index The index.
     * @return The {@link PhotonPipelienResult}.
     */
    public PhotonPipelineResult getResult(int index) {
        return m_results.get(index);
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