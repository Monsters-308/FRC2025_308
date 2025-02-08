// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;

import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HeadingConstants;
import frc.robot.Constants.ModuleConstants;
import frc.utils.FieldUtils;
import frc.utils.SwerveModule;
import frc.utils.GeneralUtils;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // Create Swerve Modules
    private final SwerveModule m_frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.KFrontLeftTurningEncoderId,
        ModuleConstants.kLeftFrontInverted);

    private final SwerveModule m_frontRight = new SwerveModule(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.KFrontRightTurningEncoderId,
        ModuleConstants.kRightFrontInverted);

    private final SwerveModule m_rearLeft = new SwerveModule(
        DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.kRearLeftTurningCanId,
        DriveConstants.KRearLeftTurningEncoderId,
        ModuleConstants.kLeftRearInverted);

    private final SwerveModule m_rearRight = new SwerveModule(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.KRearRightTurningEncoderId,
        ModuleConstants.kRightRearInverted);

    // The gyro sensor
    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
    
    // Note: the NavX takes a second to configure before it can be used. I have seen some teams create the gyro in a separate thread, which might be worth considering.

    // These values are for tracking slew rate, which lets the robot accelerate and slow down gradually
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;
    private ChassisSpeeds m_prevTarget = new ChassisSpeeds();

    // Field widget for displaying odometry
    private final Field2d m_field = new Field2d();

    // Shuffleboard objects
    private final ShuffleboardTab m_swerveTab = Shuffleboard.getTab("Swerve");
    // Add alliance widget (it's just a boolean widget but I manually change the color)
    private final SimpleWidget m_allianceWidget = m_swerveTab.add("Alliance", true); 
    // Widget for toggling limelight data
    private final SimpleWidget m_useLimelightData;
    // Widget for toggling photon data
    private final SimpleWidget m_usePhotonData;

    // Suppliers for pose estimation with vision data
    private final Supplier<Pose2d> m_visionPose;
    private final DoubleSupplier m_visionTimestamp;
    private final Supplier<Optional<EstimatedRobotPose>> m_photonEstimation;

    // Swerve pose estimator
    private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(getGyroAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        new Pose2d(),
        /**
         * VecBuilder -> Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
         * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
        */
        VecBuilder.fill(0.1, 0.1, .05),
        /**
         * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
         * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
        */
        VecBuilder.fill(2, 2, 3)
    );

    /** 
     * This controls all of the swerve modules, along with the pathplanner setup, the gyro,
     * the odometry, and the use of odometry and vision data to estimate the robot's pose.
     */
    public DriveSubsystem(Supplier<Pose2d> visionPosition, DoubleSupplier visionTimestamp, Supplier<Optional<EstimatedRobotPose>> photonEstimation) {
        m_visionPose = visionPosition;
        m_visionTimestamp = visionTimestamp;
        m_photonEstimation = photonEstimation;

        m_gyro.enableLogging(true);
    
        // Widgets for swerve module angles 
        m_swerveTab.addDouble("frontLeft angle", () -> GeneralUtils.angleConstrain(m_frontLeft.getTurningAngle().getDegrees()));
        m_swerveTab.addDouble("frontRight angle", () -> GeneralUtils.angleConstrain(m_frontRight.getTurningAngle().getDegrees()));
        m_swerveTab.addDouble("rearLeft angle", () -> GeneralUtils.angleConstrain(m_rearLeft.getTurningAngle().getDegrees()));
        m_swerveTab.addDouble("rearRight angle", () -> GeneralUtils.angleConstrain(m_rearRight.getTurningAngle().getDegrees()));
        m_swerveTab.addDouble("frontLeft test angle", () -> GeneralUtils.angleConstrain(m_frontLeft.getRelativeTurningAngle()));
        m_swerveTab.addDouble("frontRight test angle", () -> GeneralUtils.angleConstrain(m_frontRight.getRelativeTurningAngle()));
        m_swerveTab.addDouble("rearLeft test angle", () -> GeneralUtils.angleConstrain(m_rearLeft.getRelativeTurningAngle()));
        m_swerveTab.addDouble("rearRight test angle", () -> GeneralUtils.angleConstrain(m_rearRight.getRelativeTurningAngle()));

        m_swerveTab.addDouble("Desired Angle", () -> m_frontLeft.getDesiredAngle());

        // Gyro widget
        m_swerveTab.addDouble("Robot Heading", this::getHeading)
            .withWidget(BuiltInWidgets.kGyro)
            .withSize(2, 2)
            .withProperties(Map.of(
                "Counter Clockwise", true));
        
        // Field widget for displaying odometry estimation
        m_swerveTab.add("Field", m_field)
            .withSize(6, 3);
        
        m_swerveTab.addDouble("robot X", () -> getPose().getX());
        m_swerveTab.addDouble("robot Y", () -> getPose().getY());

        // // Gyro values for testing
        // swerveTab.addDouble("gyro pitch", () -> m_gyro.getPitch());
        // swerveTab.addDouble("gyro roll", () -> m_gyro.getRoll());
        
        // Configure the AutoBuilder
        AutoBuilder.configure(
            () -> FieldUtils.flipRed(getPose()), // Robot pose supplier
            (Pose2d newPose) -> resetOdometry(FieldUtils.flipRed(newPose)), // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            AutoConstants.kPathPlannerController, // Path planner controller for holonomic drive.
            AutoConstants.kPathPlannerRobotConfig, // Path planner config for robot constants
            FieldUtils::isRedAlliance, // Parameter for whether to invert the paths for red alliance (returns false if alliance is invalid)
            this // Reference to this subsystem to set requirements
        );

        m_useLimelightData = m_swerveTab.add("Limelight Data", true)
            .withWidget(BuiltInWidgets.kToggleSwitch);

        m_usePhotonData = m_swerveTab.add("Photon Vision Data", true)
            .withWidget(BuiltInWidgets.kToggleSwitch);

        GeneralUtils.configureSysID(
            m_swerveTab.getLayout("Linear SysID"), this,
            voltage -> {
                m_frontLeft.setDriveVoltage(voltage);
                m_frontRight.setDriveVoltage(voltage);
                m_rearLeft.setDriveVoltage(voltage);
                m_rearRight.setDriveVoltage(voltage);
            }
        );

        GeneralUtils.configureSysID(
            m_swerveTab.getLayout("Angular SysID"), this,
            voltage -> {
                m_frontLeft.setDriveVoltage(voltage);
                m_frontRight.setDriveVoltage(voltage.times(-1));
                m_rearLeft.setDriveVoltage(voltage);
                m_rearRight.setDriveVoltage(voltage.times(-1));
            }
        );
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
            
        // Try to add vision data to pose estimation
        double timestamp = m_visionTimestamp.getAsDouble();
        Pose2d pose = m_visionPose.get();
        
        if((pose != null) && (m_useLimelightData.getEntry().getBoolean(true))){
            m_odometry.addVisionMeasurement(pose, timestamp);
        }
        
        Optional<EstimatedRobotPose> estimationOpt = m_photonEstimation.get();
        estimationOpt.ifPresent(estimation -> {
            if (m_usePhotonData.getEntry().getBoolean(true)) {
                m_odometry.addVisionMeasurement(estimation.estimatedPose.toPose2d(), estimation.timestampSeconds);
            }
        });

        // Update field widget
        m_field.setRobotPose(FieldUtils.fieldWidgetScale(getPose()));
    
        // Widget that shows color of alliance
        if (FieldUtils.getAlliance(true) == null) {
            m_allianceWidget.withProperties(Map.of(
                    "Color when true", "Gray"
                ));
        }
        else {
            switch (FieldUtils.getAlliance(false)) {
                case Blue:
                    m_allianceWidget.withProperties(Map.of(
                        "Color when true", "Blue"
                    ));
                    break;
                
                case Red:
                    m_allianceWidget.withProperties(Map.of(
                        "Color when true", "Red"
                    ));
                    break;
            }    
        }
        
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getEstimatedPosition();
    }

    /**
     * Resets the odometry to the specified pose. Note: this also resets the angle of the robot.
     *
     * @param pose The pose to which to set the odometry.
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
     *
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
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getHeading()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );

        final double
            currentTime = WPIUtilJNI.now() * 1e-6,
            elapsedTime = currentTime - m_prevTime;

        // Rate limit if applicable
        if(rateLimit) {
            // Side effect: The velocities of targetVel are modified by this function
            GeneralUtils.rateLimitVelocity(
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
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        setHeading(0);
    }

    /**
     * Sets the robot's heading to a specific angle.
     * 
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
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return GeneralUtils.angleConstrain(
            m_odometry.getEstimatedPosition().getRotation().getDegrees()
        );
    }

    /**
     * Returns the gyro's angle adjusted for inversion.
     * @apiNote This may not be the same as getHeading() and is not constrained from -180 to 180.
     * @return The angle of the gyro adjusted for inversion.
     */
    private double getGyroAngle() {
        return m_gyro.getAngle() * (HeadingConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Returns the pitch of the ROBOT (not necessarily the gyro).
     * @return The pitch of the robot in degrees from -180 to 180.
     */
    public double getRobotPitch() {
        return m_gyro.getRoll();
    }

    /**
     * Returns the roll of the ROBOT (not necessarily the gyro).
     * @return The roll of the robot in degrees from -180 to 180.
     */
    public double getRobotRoll() {
        return m_gyro.getPitch();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (HeadingConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Used by pathplanner to figure out how quickly the robot is moving.
     * 
     * @return The robot-relative translational speeds
     */
    private ChassisSpeeds getRobotRelativeSpeeds(){
        // Uses forward kinematics to calculate the robot's speed given the states of the swerve modules.
        return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState());
    }

    /**
     * Used by pathplanner to control the robot.
     * 
     * @param speeds The velocities to move the chassis at.
     */
    private void driveRobotRelative(ChassisSpeeds speeds){
        // This takes the velocities and converts them into precentages (-1 to 1)
        drive((speeds.vxMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond), 
            (speeds.vyMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond), 
            speeds.omegaRadiansPerSecond / DriveConstants.kMaxAngularSpeed, 
            false, 
            false);
    }
}
