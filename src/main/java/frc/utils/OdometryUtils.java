package frc.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;

public class OdometryUtils {

    /**
     * Restricts a pose's translation to the confines of the field.
     * This isn't very useful for our practice field where we have no barriers,
     * but it will increase the accuracy of our odometry on the real field. 
     * @param pose The pose to be restricted.
     * @return The same pose but with restrictions applied.
     */
    public static Pose2d restrictPose(Pose2d pose){
        return new Pose2d(
            MathUtil.clamp(pose.getX(), 0, FieldConstants.kFieldWidthMeters),
            MathUtil.clamp(pose.getY(), 0, FieldConstants.kFieldHeightMeters),
            pose.getRotation()
        );
    }

    /**
     * Calculates the angle that point A has to be at in order to face
     * point B. This will be used for auto-aiming.
     * @param pointA The point that needs to face another point.
     * @param pointB The point being faced at by pointA.
     * @return The angle from point A to point B.
     */
    public static Rotation2d anglePoseToPose(Translation2d pointA, Translation2d pointB){
        double deltaX = pointB.getX() - pointA.getX();
        double deltaY = pointB.getY() - pointA.getY();

        // Use Math.atan2 to calculate the angle
        double angle = Math.atan2(deltaY, deltaX);

        return new Rotation2d(angle);
    }

    /**
     * Calculates the distance that point A is from point B
     * This will be used for auto-aiming.
     * @param pointA The starting point
     * @param pointB The ending point
     * @return The distance from point A to point B.
     */
    public static double getDistancePosToPos(Translation2d pointA, Translation2d pointB){
        double x1 = pointA.getX();
        double y1 = pointA.getY();

        double x2 = pointB.getX();
        double y2 = pointB.getY();

        double distance = Math.sqrt(Math.abs(Math.pow(x1-x2, 2)) + Math.abs(Math.pow(y1-y2, 2)));

        return distance;
    }
    
}
