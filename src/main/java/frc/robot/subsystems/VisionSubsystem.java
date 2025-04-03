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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.LoggingUtils;
import frc.robot.Constants.FieldConstants;

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

    /** An {@link ShuffleboardTab} for PhotonVision. */
    private final ShuffleboardTab m_visionTab = Shuffleboard.getTab("Vision");

    private final GenericEntry[] m_tranformationX;
    private final GenericEntry[] m_tranformationY;
    private final GenericEntry[] m_tranformationAngle;

    /**
     * Creates a new {@link VisionSubsystem}.
     */
    public VisionSubsystem() {
        m_cameras = new PhotonCamera[VisionConstants.kCameraNames.length];
        m_photonPoseEstimators = new PhotonPoseEstimator[m_cameras.length];
        m_tranformationX = new GenericEntry[m_cameras.length];
        m_tranformationY = new GenericEntry[m_cameras.length];
        m_tranformationAngle = new GenericEntry[m_cameras.length];

        for (int i = 0; i < m_cameras.length; i++) {
            m_cameras[i] = new PhotonCamera(VisionConstants.kCameraNames[i]);
            m_photonPoseEstimators[i] = new PhotonPoseEstimator(
                FieldConstants.kAprilTagFieldLayout,
                VisionConstants.kPoseStrategy,
                VisionConstants.kRobotToCameraTransformations[i]
            );

            ShuffleboardLayout cameraLayout = m_visionTab.getLayout("\"" + VisionConstants.kCameraNames[i] + "\" Transformation", BuiltInLayouts.kList);

            m_tranformationX[i] = cameraLayout.add("X", VisionConstants.kRobotToCameraTransformations[i].getX()).getEntry();
            m_tranformationY[i] = cameraLayout.add("Y", VisionConstants.kRobotToCameraTransformations[i].getY()).getEntry();
            m_tranformationAngle[i] = cameraLayout.add("Angle",
                Units.radiansToDegrees(VisionConstants.kRobotToCameraTransformations[i].getRotation().getZ())
            ).getEntry();

            LoggingUtils.logCamera(m_cameras[i]);
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
     * Gets the {@link PhotonPipelineResult} objects for every camera.
     * @return The {@link PhotonPipelienResult} objects.
     */
    public PhotonPipelineResult[] getResults() {
        return m_results;
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
     * @param index The pipeline index to set the camera to.
     */
    public void setPipeline(int camera, int index) {
        m_cameras[camera].setPipelineIndex(index);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < m_cameras.length; i++) {
            if (!m_cameras[i].isConnected()) {
                m_results[i] = null;
                continue;
            }

            List<PhotonPipelineResult> results = m_cameras[i].getAllUnreadResults();

            if (!results.isEmpty()) {
                m_results[i] = results.get(results.size() - 1);
            } else {
                m_results[i] = null;
            }
        }

        for (int i = 0; i < m_cameras.length; i++) {
            Transform3d transform = new Transform3d(
                m_tranformationX[i].getDouble(VisionConstants.kRobotToCameraTransformations[i].getX()),
                m_tranformationY[i].getDouble(VisionConstants.kRobotToCameraTransformations[i].getY()),
                0,
                new Rotation3d(
                    0, 0,
                    Units.degreesToRadians(m_tranformationAngle[i].getDouble(
                        Units.radiansToDegrees(VisionConstants.kRobotToCameraTransformations[i].getRotation().getZ())
                    ))
                )
            );

            m_photonPoseEstimators[i].setRobotToCameraTransform(transform);
        }
    }
}