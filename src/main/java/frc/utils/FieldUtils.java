package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class FieldUtils {
    /**
     * Returns the Alliance color.
     * @param nullable (Optional) Whether null should be returned if the alliance is invalid. 
     * If set to false, Blue is returned instead of null.
     * @return The Alliance color (or null/Blue if invalid).
     */
    public static Alliance getAlliance(boolean nullable){
        Alliance alliance = DriverStation.getAlliance().orElse(null);
        if (nullable || alliance != null){
            return alliance;
        }

        // Return Blue if alliance is null and nullable is false
        return Alliance.Blue;
    }

    /**
     * Returns the Alliance color.
     * @return The Alliance color (or Blue if invalid).
     */
    public static Alliance getAlliance(){
        return getAlliance(false);
    }

    /**
     * Returns whether or not the robot is on the red alliance.
     * @return true if on red alliance, false if on blue alliance or invalid.
     */
    public static boolean isRedAlliance(){
        return getAlliance() == Alliance.Red;
    }

    /**
     * Adjusts a Pose2d so that it'll display properly on the Field widget.
     * This also takes into account the alliance color.
     * @param pose The pose object relative to the blue side (in meters).
     * @return The new pose object scaled.
     */
    public static Pose2d fieldWidgetScale(Pose2d pose){
        pose = flipGlobalBlue(pose);

        // linearly scale widget x and y
        // NOTE: We are doing this because shuffleboard's field widget is bugged. You would 
        // normally never have to do this.
        pose = new Pose2d(
            pose.getX() * ((16.9 - 1.05) / 16.54) + 1.05, // 1.05 to 16.9
            pose.getY() * (7.9 / 8.2) + 0.3, // 0.3 to 8.2
            pose.getRotation()
        );
        
        return pose;
    }

    /**
     * Flips a translation object's Y value to line up on the red side if the robot is on the red alliance.
     * @param position A translation object for the blue side.
     * @return The translation object mirrored for the red side.
     */
    public static Translation2d flipRed(Translation2d position){
        if (isRedAlliance()){
            return new Translation2d(
                position.getX(),
                FieldConstants.kFieldHeightMeters-position.getY()
            );  
        }
        return position;
    }

    /**
     * Mirrors a pose2d object's Y value and angle to line up on the red side if the robot is on the red alliance.
     * @param position A pose2d object for the blue side.
     * @return The translation object mirrored for the red side.
     */
    public static Pose2d flipRed(Pose2d position){
        if (isRedAlliance()){
            return new Pose2d(
                flipRed(position.getTranslation()),
                flipRedAngle(position.getRotation())
            );
        }
        return position;
    }

    /**
     * Flips a y coordinate so that it'll line up on the red side if the robot is on the red alliance.
     * @param yPosition A number used to represent the y coordinate of a position (in meters).
     * @return The y value mirrored down the middle of the field (in meters).
     */
    public static double flipRedY(double yPosition){
        if (isRedAlliance()){
            return FieldConstants.kFieldHeightMeters - yPosition;
        }
        return yPosition;
    }

    /**
     * Mirrors a Rotation2d object so that left becomes right if the robot is on the red alliance.
     * This is equivalent to multiplying the angle by -1.
     * @param angle A rotation2d object. 
     * @return The same object but flipped.
     */
    public static Rotation2d flipRedAngle(Rotation2d angle){
        if (isRedAlliance()){
            return angle.times(-1);
        }
        return angle;
    }

    /**
     * Mirrors an angle so that left becomes right if the robot is on the red alliance.
     * This is equivalent to multiplying the angle by -1.
     * @param angle An angle in degrees or radians, as long as it's from -180 to 180 or -Pi to Pi.
     * @return The flipped angle.
     */
    public static double flipRedAngle(double angle){
        if (isRedAlliance()){
            return -angle;
        }
        return angle;
    }

    /**
     * Similar to redWidgetFlip, this flips both the x, y, and rotation in order to 
     * keep a global blue alliance origin. This can be used for converting into and 
     * out of a global blue alliance origin.
     * @return The pose flipped if on red alliance.
     */
    public static Pose2d flipGlobalBlue(Pose2d robotPose){
        if(isRedAlliance()){
            return new Pose2d(
                FieldConstants.kFieldWidthMeters - robotPose.getX(), 
                FieldConstants.kFieldHeightMeters - robotPose.getY(), 
                Rotation2d.fromDegrees(robotPose.getRotation().getDegrees() + 180));
        }
        return robotPose;
    }
}
