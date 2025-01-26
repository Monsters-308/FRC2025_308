package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConstants;

public class PhotonSubsystem extends SubsystemBase {

    public PhotonSubsystem() {
        PhotonCamera camera = new PhotonCamera(PhotonConstants.kCameraName);
        
        
    }

    public void getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose){
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }


    

}