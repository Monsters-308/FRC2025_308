// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class FieldUtils {
    private FieldUtils() {}

    /**
     * Returns the Alliance color.
     * @param nullable (Optional) Whether null should be returned if the alliance is invalid. 
     * If set to false, Blue is returned instead of null.
     * @return The Alliance color (or null/Blue if invalid).
     */
    public static Alliance getAlliance(boolean nullable) {
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
    public static Alliance getAlliance() {
        return getAlliance(false);
    }

    /**
     * Returns whether or not the robot is on the red alliance.
     * @return True if on red alliance, false if on blue alliance or invalid.
     */
    public static boolean isRedAlliance() {
        return getAlliance().equals(Alliance.Red);
    }

    /**
     * Returns whether or not the robot is on the blue alliance.
     * @return True if on blue alliance or invalid, false if on red alliance.
     */
    public static boolean isBlueAlliance() {
        return getAlliance().equals(Alliance.Blue);
    }

    /**
     * Rotates a Translation2d's coordinates around the origin if the robot is on the red
     * alliance. This converts between a global blue origin and an alliance-relative origin.
     * @param position Coordinates using one type of orign.
     * @return Coordinates using the opposite type of origin.
     */
    public static Translation2d convertAllianceRelative(Translation2d position) {
        if (isRedAlliance()) {
            return new Translation2d(
                FieldConstants.kFieldWidthMeters - position.getX(),
                FieldConstants.kFieldHeightMeters - position.getY()
            );
        }
        return position;
    }

    /**
     * Rotates a Rotation2d by 180 degrees if the robot is on the red alliance.
     * This converts between a global blue origin and an alliance-relative origin.
     * @param angle A Rotation2d object using one type of orign. 
     * @return A Rotation2d object using the other type of orign. 
     */
    public static Rotation2d convertAllianceRelative(Rotation2d angle) {
        if (isRedAlliance()) {
            return Rotation2d.fromDegrees(angle.getDegrees() + 180);
        }
        return angle;
    }

    /**
     * Rotates a Pose2d around the origin if the robot is on the red alliance.
     * This converts between a global blue origin and an alliance-relative origin.
     * @param position A Pose2d using one type of orign.
     * @return A Pose2d using the opposite type of origin.
     */
    public static Pose2d convertAllianceRelative(Pose2d position) {
        if (isRedAlliance()) {
            return new Pose2d(
                convertAllianceRelative(position.getTranslation()),
                convertAllianceRelative(position.getRotation())
            );
        }
        return position;
    }

    /**
     * Flips a translation object's Y value to line up on the blue side if the robot is on the blue alliance.
     * @param position A translation object for the red side.
     * @return The translation object mirrored for the blue side.
     */
    public static Translation2d flipBlue(Translation2d position) {
        flip(convertAllianceRelative(position));
        return position;
    }

    /**
     * Mirrors a pose2d object's Y value and angle to line up on the blue side if the robot is on the blue alliance.
     * @param position A pose2d object for the red side.
     * @return The position object mirrored for the blue side.
     */
    public static Pose2d flipBlue(Pose2d position) {
        flip(convertAllianceRelative(position));
        return position;
    }

    /**
     * Flips a position to the other alliance.
     * @param pose The position to flip.
     * @return The flipped position.
     */
    public static Pose2d flip(Pose2d pose) {
        return new Pose2d(
            FieldConstants.kFieldWidthMeters - pose.getX(),
            FieldConstants.kFieldHeightMeters - pose.getY(),
            Rotation2d.fromDegrees(pose.getRotation().getDegrees() + 180)
        );
    }

    /**
     * Flips a translation to the other alliance.
     * @param translation The translation to flip.
     * @return The flipped translation.
     */
    public static Translation2d flip(Translation2d translation) {
        return new Translation2d(
            FieldConstants.kFieldWidthMeters - translation.getX(),
            FieldConstants.kFieldHeightMeters - translation.getY()
        );
    }
}
