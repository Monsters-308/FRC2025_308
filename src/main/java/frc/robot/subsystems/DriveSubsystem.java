// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.WPIUtilJNI;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.calculation.CalculateStandardDeviation;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.LoggingUtils;
import frc.robot.utils.SwerveModule;
import frc.robot.utils.Utils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets ;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem that controls all of the {@link SwerveModule} objects, along with the pathplanner setup, the gyro,
     * the odometry, and the use of odometry and vision data to estimate the robot's position.
 */
public class DriveSubsystem extends SubsystemBase {
    // Create Swerve Modules

    /** The {@link SwerveModule} for the front left wheel. */
    private final SwerveModule m_frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontLeftTurningEncoderId,
        ModuleConstants.kLeftFrontInverted);

    /** The {@link SwerveModule} for the front right wheel. */
    private final SwerveModule m_frontRight = new SwerveModule(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kFrontRightTurningEncoderId,
        ModuleConstants.kRightFrontInverted);

    /** The {@link SwerveModule} for the rear left wheel. */
    private final SwerveModule m_rearLeft = new SwerveModule(
        DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.kRearLeftTurningCanId,
        DriveConstants.kRearLeftTurningEncoderId,
        ModuleConstants.kLeftRearInverted);

    /** The {@link SwerveModule} for the rear right wheel. */
    private final SwerveModule m_rearRight = new SwerveModule(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.kRearRightTurningEncoderId,
        ModuleConstants.kRightRearInverted);

    /** An {@link AHRS} object that represents the robot gyro. */
    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
    
    // Note: the NavX takes a second to configure before it can be used. I have seen some teams create the gyro in a separate thread, which might be worth considering.

    /** The previous time for slew rate calculation. */
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;
    /** The previous {@link ChassisSpeeds} target for slew rate calculation. */
    private ChassisSpeeds m_prevTarget = new ChassisSpeeds();

    /** The Field widget for displaying odometry. */
    private final Field2d m_field = new Field2d();

    // Shuffleboard objects

    /** A {@link ShuffleboardTab} for swerve drive. */
    private final ShuffleboardTab m_swerveTab = Shuffleboard.getTab("Swerve");
    /** {@link SimpleWidget} for toggling Photon Vision data. */
    private final SimpleWidget m_usePhotonData;

    /** The {@link VisionSubsystem} of the robot. */
    private final VisionSubsystem m_visionSubsystem;

    /** Offset for the heading used for determining field relative controls.
     * Allows for reseting the controls when the controls are disoriented. */
    private double m_fieldRelativeHeadingOffset = 0;

    /** A {@link SwerveDrivePoseEstimator} for estimating the position of the robot. */
    @SuppressWarnings("unchecked")
    private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(getGyroAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        new Pose2d(
            FieldConstants.kFieldWidthMeters / 2,
            FieldConstants.kFieldHeightMeters / 2,
            Rotation2d.fromDegrees(180)
        ),
        DriveConstants.kStateStandardDeviations,
        (Vector<N3>)VisionConstants.kVisionStandardDeviations[0]
    );

    /**
     * Constructs a {@link DriveSubsystem} to control all of the {@link SwerveModule} objects, along with the pathplanner setup, the gyro,
     * the odometry, and the use of odometry and vision data to estimate the robot's position.
     */
    public DriveSubsystem(VisionSubsystem photonSubsystem) {
        m_visionSubsystem = photonSubsystem;

        LoggingUtils.logNavX(m_gyro);
        m_gyro.enableLogging(false);
    
        // Widgets for swerve module angles 
        m_swerveTab.addDouble("frontLeft angle", () -> Utils.roundToNearest(Utils.angleConstrain(m_frontLeft.getTurningAngle().getDegrees()), 2));
        m_swerveTab.addDouble("frontRight angle", () -> Utils.roundToNearest(Utils.angleConstrain(m_frontRight.getTurningAngle().getDegrees()), 2));
        m_swerveTab.addDouble("rearLeft angle", () -> Utils.roundToNearest(Utils.angleConstrain(m_rearLeft.getTurningAngle().getDegrees()), 2));
        m_swerveTab.addDouble("rearRight angle", () -> Utils.roundToNearest(Utils.angleConstrain(m_rearRight.getTurningAngle().getDegrees()), 2));

        // Gyro widget
        m_swerveTab.addDouble("Robot Heading", () -> Utils.roundToNearest(getHeading(), 2));
        
        // Field widget for displaying odometry estimation
        m_swerveTab.add("Field", m_field)
            .withSize(6, 3);
        
        m_swerveTab.addDouble("robot X", () -> Utils.roundToNearest(getPose().getX(), 2));
        m_swerveTab.addDouble("robot Y", () -> Utils.roundToNearest(getPose().getY(), 2));

        m_swerveTab.addBoolean("Alliance", FieldUtils::isBlueAlliance);

        // // Gyro values for testing
        m_swerveTab.addDouble("gyro pitch", () -> Utils.roundToNearest(m_gyro.getPitch(), 2));
        m_swerveTab.addDouble("gyro roll", () -> Utils.roundToNearest(m_gyro.getRoll(), 2));
        
        // Configure the AutoBuilder
        AutoBuilder.configure(
            () -> FieldUtils.convertAllianceRelative(getPose()), // Robot pose supplier
            (Pose2d newPose) -> resetOdometry(FieldUtils.convertAllianceRelative(newPose)), // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            AutonConstants.kPathPlannerController, // Path planner controller for holonomic drive
            AutonConstants.kPathPlannerRobotConfig, // Path planner config for robot constants
            FieldUtils::isRedAlliance, // Parameter for whether to invert the paths for red alliance (returns false if alliance is invalid)
            this // Reference to this subsystem to set requirements
        );

        m_usePhotonData = m_swerveTab.addPersistent("Use PhotonVision Data", true)
            .withWidget(BuiltInWidgets.kToggleSwitch);

        Utils.configureSysID(
            m_swerveTab.getLayout("Linear SysID", BuiltInLayouts.kList), this,
            voltage -> {
                m_frontLeft.setDriveVoltage(voltage);
                m_frontRight.setDriveVoltage(voltage);
                m_rearLeft.setDriveVoltage(voltage);
                m_rearRight.setDriveVoltage(voltage);
            }
        );

        Utils.configureSysID(
            m_swerveTab.getLayout("Angular SysID", BuiltInLayouts.kList), this,
            () -> {
                SwerveModuleState turningLeftState = new SwerveModuleState();
                turningLeftState.angle = Rotation2d.fromDegrees(-45);
                SwerveModuleState turningRightState = new SwerveModuleState();
                turningRightState.angle = Rotation2d.fromDegrees(45);

                setModuleStates(new SwerveModuleState[] {
                    turningLeftState,
                    turningRightState,
                    turningLeftState,
                    turningRightState
                });
            },
            voltage -> {
                m_frontLeft.setDriveVoltage(voltage);
                m_frontRight.setDriveVoltage(voltage.times(-1));
                m_rearLeft.setDriveVoltage(voltage);
                m_rearRight.setDriveVoltage(voltage.times(-1));
            }
        );
        
        GenericEntry entry = m_swerveTab.add("StdDev", 0).getEntry();

        m_swerveTab.add("Angle StdDev", new CalculateStandardDeviation(this::getHeading, entry::setDouble, entry::setDouble));
        SmartDashboard.putNumber("Pose Variation", 0);

        ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");

        // for (int i = 0; i < VisionConstants.kCameraNames.length; i++) {
        //     m_distances[i] = visionTab.add("Distance to \"" + VisionConstants.kCameraNames[i] +"\"", 0).getEntry();
        // }

        // m_stdDevX = visionTab.add("StdDev X", VisionConstants.kVisionStandardDeviations[0].get(0)).getEntry();
        // m_stdDevY = visionTab.add("StdDev Y", VisionConstants.kVisionStandardDeviations[0].get(1)).getEntry();
    }

    @Override
    public void periodic() {
        // Update pose estimation with odometry data
        m_odometry.update(
            Rotation2d.fromDegrees(getGyroAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            });
        
        // Save the current pose to compare it to the vision-adjusted pose
        Pose2d oldPose = m_odometry.getEstimatedPosition();
        
        if (m_usePhotonData.getEntry().getBoolean(true)) {
            PhotonPipelineResult[] results = m_visionSubsystem.getResults();
            EstimatedRobotPose[] estimations = m_visionSubsystem.getEstimations();

            for (int i = 0; i < estimations.length; i++) {
                if (estimations[i] == null) continue;

                double dstToAprilTag = results[i].getBestTarget().getBestCameraToTarget().getTranslation().getDistance(Translation3d.kZero);

                // m_distances[i].setDouble(Utils.roundToNearest(dstToAprilTag, 2));

                Vector<N3> stdDev = VecBuilder.fill(
                    DriveConstants.kVisionStandardDeviationMultipler * (0.987 - 2.23 * dstToAprilTag + 1.32 * Math.pow(dstToAprilTag, 2)),
                    DriveConstants.kVisionStandardDeviationMultipler * (0.997 - 2.23 * dstToAprilTag + 1.32 * Math.pow(dstToAprilTag, 2)),
                    VisionConstants.kVisionStandardDeviations[i].get(2)
                );

                // Don't scale the heading standard deviation
                // stdDev.set(2, 0, VisionConstants.kVisionStandardDeviations[i].get(2));

                m_odometry.addVisionMeasurement(FieldUtils.convertAllianceRelative(estimations[i].estimatedPose.toPose2d()), estimations[i].timestampSeconds, stdDev);
            }
        }

        // Reset field relative offset to zero when FMS is connected
        if (DriverStation.isFMSAttached()) {
            m_fieldRelativeHeadingOffset = 0;
        }

        // Display variation in pose (in inches)
        SmartDashboard.putNumber("Pose Variation", 
            Units.metersToInches(
                Utils.getDistancePosToPos(oldPose.getTranslation(), m_odometry.getEstimatedPosition().getTranslation())
            )
        );

        // Update field widget
        m_field.setRobotPose(FieldUtils.convertAllianceRelative(getPose())); // Convert back to global blue origin.
    }

    /**
     * Returns the currently-estimated positon of the robot.
     * @return The position as a {@link Pose2d} object.
     */
    public Pose2d getPose() {
        return m_odometry.getEstimatedPosition();
    }

    /**
     * Resets the odometry to the specified pose. Note: this also resets the angle of the robot.
     * @param pose The pose to which to set the odometry as a {@link Pose2d} object.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            Rotation2d.fromDegrees(getGyroAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
            pose);
    }

    /**
     * Method to drive the robot using joystick info.
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     * @param rateLimit Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        
        // Convert the commanded speeds into the correct units for the drivetrain
        xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        rot *= DriveConstants.kMaxAngularSpeed;

        // Get the target chassis speeds relative to the robot
        final ChassisSpeeds targetVel = (fieldRelative ?
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getHeading() - m_fieldRelativeHeadingOffset))
                : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );

        final double
            currentTime = WPIUtilJNI.now() * 1e-6,
            elapsedTime = currentTime - m_prevTime;

        // Rate limit if applicable
        if(rateLimit) {
            // Side effect: The velocities of targetVel are modified by this function
            Utils.rateLimitVelocity(
                targetVel, m_prevTarget, elapsedTime,
                DriveConstants.kMagnitudeSlewRate, DriveConstants.kRotationalSlewRate
            );
        }

        m_prevTime = currentTime;
        m_prevTarget = targetVel;

        // Use the DriveKinematics to calculate the module states
        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetVel);

        // Normalizes the wheel speeds (makes sure none of them go above the max speed)
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

        // Set the modules to their desired states
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets {@link SwerveModuleState} objects as the desired states.
     * @param desiredStates The desired {@link SwerveModule} states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /**
     * Resets the field relative controls for swerve such that the current heading of the robot is considered forward.
     * This does nothing when FMS is connected, as on a field, the field relative control directon will always be correct.
     */
    public void resetFieldRelative() {
        if (!DriverStation.isFMSAttached()) {
            m_fieldRelativeHeadingOffset = getHeading();
        }
    }

    /**
     * Sets the robot's heading to a specific angle.
     * @param angle The angle (in degrees) to set the robot's heading to.
     */
    public void setHeading(double angle) {
        m_odometry.resetPosition(
            Rotation2d.fromDegrees(getGyroAngle()), 
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            }, 
            new Pose2d(
                getPose().getTranslation(),
                Rotation2d.fromDegrees(angle)
            )
        );
    }

    /**
     * Returns the heading of the robot.
     * @return The robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Utils.angleConstrain(
            m_odometry.getEstimatedPosition().getRotation().getDegrees()
        );
    }

    /**
     * Returns the gyro's angle adjusted for inversion.
     * @apiNote This may not be the same as getHeading() and is not constrained from -180 to 180.
     * @return The angle of the gyro adjusted for inversion.
     */
    private double getGyroAngle() {
        return m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Returns the pitch of the <i>robot</i> (not necessarily the gyro).
     * @return The pitch of the robot in degrees from -180 to 180.
     */
    public double getRobotPitch() {
        return m_gyro.getRoll();
    }

    /**
     * Returns the roll of the <i>robot</i> (not necessarily the gyro).
     * @return The roll of the robot in degrees from -180 to 180.
     */
    public double getRobotRoll() {
        return m_gyro.getPitch();
    }

    /**
     * Returns the turn rate of the robot.
     * @return The turn rate of the robot, in degrees per second.
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Used by pathplanner to figure out how quickly the robot is moving.
     * @return The robot-relative translational speeds as a {@link ChassisSpeeds} object.
     */
    private ChassisSpeeds getRobotRelativeSpeeds() {
        // Uses forward kinematics to calculate the robot's speed given the states of the swerve modules.
        return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState());
    }

    /**
     * Used by PathPlanner to control the robot.
     * @param speeds The velocities to move the chassis at as a {@link ChassisSpeeds} object.
     */
    private void driveRobotRelative(ChassisSpeeds speeds) {
        // This takes the velocities and converts them into precentages (-1 to 1)
        drive((speeds.vxMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond), 
            (speeds.vyMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond), 
            speeds.omegaRadiansPerSecond / DriveConstants.kMaxAngularSpeed, 
            false, 
            false);
    }

    /**
     * This sets the idlemode.
     */
    public void setIdleMode(IdleMode idleMode) {
        for (SwerveModule module : new SwerveModule[] { m_frontLeft, m_frontRight, m_rearLeft, m_rearRight }) {
            module.setIdleMode(idleMode);
        }
    }
}
