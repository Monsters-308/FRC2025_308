// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConstants;

/**
 * Subsystem that interfaces with PhotonVision for april tag position estimation.
 */
public class VisionSubsystem extends SubsystemBase {
    /** The {@link PhotonCamera} that represents the robot camera. */
    private final PhotonCamera[] m_cameras;

    /** The {@link PhotonPoseEstimator} used to estimate the robot position using camera results. */
    private final PhotonPoseEstimator[] m_photonPoseEstimators;

    /** An array of results from the cameras. */
    private final PhotonPipelineResult[] m_results;

    public VisionSubsystem() {
        m_cameras = new PhotonCamera[PhotonConstants.kCameraNames.length];
        m_photonPoseEstimators = new PhotonPoseEstimator[m_cameras.length];

        for (int i = 0; i < m_cameras.length; i++) {
            m_cameras[i] = new PhotonCamera(PhotonConstants.kCameraNames[i]);
            m_photonPoseEstimators[i] = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(PhotonConstants.kField),
                PhotonConstants.kPoseStrategy,
                PhotonConstants.kRobotToCameraTransformations[i]
            );
        }

        m_results = new PhotonPipelineResult[m_cameras.length];
    }

    /**
     * Gets the current position estimations from PhotonVision.
     * @return An array of {@link EstimatedRobotPose} objects that contain an estimated {@link Pose2d} and timestamp for each camera.
     * If an estimation is not present, either because no april tags are in view, or there are no new camera results, it will be <code>null</code>.
     */
    public EstimatedRobotPose[] getEstimations() {
        EstimatedRobotPose[] estimations = new EstimatedRobotPose[m_cameras.length];

        for (int i = 0; i < m_cameras.length; i++) {
            PhotonPipelineResult result = m_results[i];

            if (result == null) continue;

            Optional<EstimatedRobotPose> estimation = m_photonPoseEstimators[i].update(result);
            
            estimations[i] = estimation.orElse(null);
        }

        return estimations;
    }

    /**
     * Gets the {@link PhotonPipelineResult} for the specified camera.
     * @param camera The index of the camera.
     * @return The {@link PhotonPipelienResult}.
     */
    public PhotonPipelineResult getLatestResult(int camera) {
        return m_results[camera];
    }

    /**
     * Sets the driver mode of the specified camera.
     * @param camera The index of the camera.
     * @param mode True to turn on driver mode, false to turn off driver mode.
     */
    public void setDriverMode(int camera, boolean mode) {
        m_cameras[camera].setDriverMode(mode);
    }

    /**
     * Sets the pipeline of the specified camera.
     * @param camera The index of the camera.
     * @param mode The pipeline index to set the camera to.
     */
    public void setPipeline(int camera, int index) {
        m_cameras[camera].setPipelineIndex(index);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < m_cameras.length; i++) {
            List<PhotonPipelineResult> results = m_cameras[i].getAllUnreadResults();

            if (!results.isEmpty()) {
                m_results[i] = results.get(results.size() - 1);
            } else {
                m_results[i] = null;
            }
        }
    }
}