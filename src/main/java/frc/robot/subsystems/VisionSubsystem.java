package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.utils.FieldUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class VisionSubsystem extends SubsystemBase {
    final private NetworkTableEntry ty;
    final private NetworkTableEntry tx; 
    final private NetworkTableEntry tv;
    final private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    // Field to visualize april tag detection
    private final Field2d m_field = new Field2d();
    private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");

    /**
     * Manages receiving and sending out data to the limelight using networktables.
     * We have code for switching pipelines and getting reflective tape data,
     * but it's kind of useless since we're only using the april tag pipeline.
     */
    public VisionSubsystem(){
        ty = limelightTable.getEntry("ty");
        tx = limelightTable.getEntry("tx");
        tv = limelightTable.getEntry("tv");
        setPipeline(VisionConstants.kDefaultPipeline);

        visionTab.addInteger("Pipeline", () -> getPipeline());
        visionTab.add("Vision Estimates", m_field)
            .withSize(7, 4);
    }

    //tv = valid targets
    //tx horizontal offset from crosshair to target
    //ty vertical offset from crosshair to target
    //ta = target area 0% to 100%
    
    /** Sets the limelight to a specific pipeline using network tables. */
    public void setPipeline(int pipeline){
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    /** Returns the limelight's current pipeline. Returns -1 if no pipeline entry exists. */
    public int getPipeline(){
        return ((Double)limelightTable.getEntry("pipeline").getNumber(-1)).intValue();
    }

    /**
     * Returns the horizontal offset from the crosshair to the target. Returns 0 if the target can't be found.
     */
    public double getX(){
        return tx.getDouble(0.0);
    }

    /**
     * Returns the vertical offset from the crosshair to the target. Returns 0 if the target can't be found.
     */
    public double getY(){
        return ty.getDouble(0.0);
    }
    
    /**
     * Returns the number of valid targets detected by the limelight. Returns 0 if no targets are found.
     */
    public int getTargets(){
        return (int)tv.getDouble(0);
    }

    public double getTimeStamp(){
        // Check if the limelight is actually viewing an apriltag
        if((getPipeline() == VisionConstants.kAprilTagPipeline) && (getTargets() > 0)) {

            // Get pose data from networktables
            double[] robotPosArrary = limelightTable.getEntry(
                FieldUtils.isRedAlliance() ?
                    "botpose_wpired" :
                    "botpose_wpiblue"
            ).getDoubleArray(new double[6]);

            // Return the latency to calculate april tag data
            return (robotPosArrary[6]);
        }
        return 0;
    }

    public double getTimeStampEstimator(){
        // Check if the limelight is actually viewing an apriltag
        if((getPipeline() == VisionConstants.kAprilTagPipeline) && (getTargets() > 0)) {
            return (getLastTimeStamp() / 1e6 - getTimeStamp() / 1e3);
        }
        return 0;
    }

    public double getLastTimeStamp(){
        // Check if the limelight is actually viewing an apriltag
        if((getPipeline() == VisionConstants.kAprilTagPipeline) && (getTargets() > 0)) {

            // Get pose data from networktables
            double robotLastTimeStamp = limelightTable.getEntry(
                FieldUtils.isRedAlliance() ?
                    "botpose_wpired" :
                    "botpose_wpiblue"
            ).getLastChange();

            // Turn pose data into pose2d object
            return (robotLastTimeStamp);
        }
        return 0;
    }

    /**
     * Uses whatever april tag is in front of the limelight to estimate the robot's position on the field. 
     * Returns null if no april tag is in view.
     * @return The position of the robot according to apriltags as a Pose2d object, or null. 
     */
    public Pose2d getRobotPosition(){
        // Check if the limelight is actually viewing an apriltag
        if((getPipeline() == VisionConstants.kAprilTagPipeline) && (getTargets() > 0)) {

            // Get pose data from networktables
            double[] robotPosArrary = limelightTable.getEntry(
                FieldUtils.isRedAlliance() ?
                    "botpose_wpired" :
                    "botpose_wpiblue"
            ).getDoubleArray(new double[6]);

            // Turn pose data into pose2d object
            return new Pose2d(robotPosArrary[0], robotPosArrary[1], Rotation2d.fromDegrees(robotPosArrary[5]));
        }
        return null;
    }

    @Override
    public void periodic(){
        // Display april tag data on a field widget for testing
        Pose2d limelightPose = getRobotPosition();
        if(limelightPose != null){
            m_field.setRobotPose(FieldUtils.fieldWidgetScale(limelightPose));
        }        
    }

}