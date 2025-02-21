// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.AlgaeIntakeRollerSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.utils.LoggingUtils;
import frc.robot.utils.SwerveModule;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    private Constants() {}

    /**
     * Constants that describe physical aspects of the entire robot.
     */
    public static final class RobotConstants {
        private RobotConstants() {}

        /** Mass of the robot in kilograms. */
        public static final double kRobotMassKG = 68;

        /** Moment of inertia of the robot. */
        public static final double kRobotMOI = 20;
    }

    /**
     * Describe how the {@link DriveSubsystem} should move the robot.
     */
    public static final class DriveConstants {
        private DriveConstants() {}

        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds

        /** The highest allowed speed of the drive train. */
        public static final double kMaxSpeedMetersPerSecond = 4.6;

        /** The highest allowed rotational speed of the drive train. */
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        /** The highest acceleration allowed in the drive train */
        public static final double kMagnitudeSlewRate = 5 * kMaxSpeedMetersPerSecond; // meters per second^2

        /** The highest rotational acceleration allowed in the drive train */
        public static final double kRotationalSlewRate = 5 * kMaxAngularSpeed; // radians per second^2

        // Chassis configuration

        /** Distance between centers of right and left wheels on robot. */
        public static final double kTrackWidth = Units.inchesToMeters(23);

        /** Distance between front and back wheels on robot. */
        public static final double kWheelBase = Units.inchesToMeters(23.125);

        /** A kinematics object that calculates how the robot should move using the positions of the modules on the robot. */
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    

        // SPARK MAX CAN IDs

        /** CAN ID of the front left driving {@link SparkMax}. */
        public static final int kFrontLeftDrivingCanId = 8; 
        /** CAN ID of the rear left driving {@link SparkMax}. */
        public static final int kRearLeftDrivingCanId = 6;
        /** CAN ID of the front right driving {@link SparkMax}. */
        public static final int kFrontRightDrivingCanId = 2;
        /** CAN ID of the rear lefrightt driving {@link SparkMax}. */
        public static final int kRearRightDrivingCanId = 4;

        /** CAN ID of the front left turning {@link SparkMax}. */
        public static final int kFrontLeftTurningCanId = 7;
        /** CAN ID of the rear left turning {@link SparkMax}. */
        public static final int kRearLeftTurningCanId = 5;
        /** CAN ID of the front right turning {@link SparkMax}. */
        public static final int kFrontRightTurningCanId = 1;
        /** CAN ID of the rear right turning {@link SparkMax}. */
        public static final int kRearRightTurningCanId = 3;

        // Encoder CAN Ids

        /** CAN ID of the front left turning {@link CANcoder}. */
        public static final int kFrontLeftTurningEncoderId = 14;
        /** CAN ID of the front right turning {@link CANcoder}. */
        public static final int kFrontRightTurningEncoderId = 12;
        /** CAN ID of the rear left turning {@link CANcoder}. */
        public static final int kRearLeftTurningEncoderId = 13;
        /** CAN ID of the rear right turning {@link CANcoder}. */
        public static final int kRearRightTurningEncoderId = 11;

        /** Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
         * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians. */
        public static final Vector<N3> kStateStandardDeviations = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

        /** Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
         * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians. */
        public static final Vector<N3> kVisionStandardDeviations = VecBuilder.fill(2, 2, 3);

        /** A multipler that controls how much the distance should scale the {@link DriveConstants#kVisionStandardDeviations}. */
        public static final double kVisionStandardDeviationMultipler = 1;
    }

    /**
     * Constants related to the individual {@link SwerveModule} objects and not to the drive subsystem itself.
     */
    public static final class ModuleConstants {
        private ModuleConstants() {}

        // Calculations required for driving motor conversion factors and feedforward

        /** The diameter of a wheel in meter. */
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        /** The radius of the wheel in meters. */
        public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;
        /** The circumference of the wheel in meters. */
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        /** The {@link DCMotor} that represents a module. */
        public static final DCMotor kMotorGearboxConfig = DCMotor.getNEO(1);

        /** The coefficient of friction between the drive wheel and the carpet */
        public static final double kWheelCOF = 1;

        // The L2 MK4 and MK4i modules have a gear ratio of 6.75:1 on the drive wheels.

        /** How much gear ratio reduces the amount of wheel revolutions relative to the amount of motor revolutions. */
        public static final double kDrivingMotorReduction = 6.75;
        /** The maximum amount of speed a module can drive when running at full power. */
        public static final double kDriveWheelFreeSpeedMetersPerSecond = (NEOMotorConstants.kFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;

        /** Converts motor revolutions to meters traveled. */
        public static final double kDrivingEncoderPositionFactor = kWheelCircumferenceMeters
            / kDrivingMotorReduction; // meters
        /** Converts motor revolutions per minute to meters per second. */
        public static final double kDrivingEncoderVelocityFactor = (kWheelCircumferenceMeters
            / kDrivingMotorReduction) / 60.0; // meters per second

        /** How much gear ratio reduces the amount of wheel revolutions relative to the amount of motor revolutions. */
        public static final double kTurningMotorReduction = 150.0 / 7;

        /** Converts motor revolutions to angle in radians. */
        public static final double kTurningEncoderPositionFactor = (2 * Math.PI) / kTurningMotorReduction; // radians
        /** Converts motor revolutions per minute to radians per second. */
        public static final double kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor / 60.0; // radians per second

        /** Smallest angle in radians of turning encoder for value wrapping. */
        public static final double kTurningEncoderPositionPIDMinInput = -Math.PI; // radians
        /** Largest angle in radians of turning encoder for value wrapping. */
        public static final double kTurningEncoderPositionPIDMaxInput = Math.PI; // radians

        /** The P for the driving PID. */
        public static final double kDrivingP = 0.04;
        /** The I for the driving PID. */
        public static final double kDrivingI = 0;
        /** The D for the driving PID. */
        public static final double kDrivingD = 0;
        /** The feedforward for the REV {@link ClosedLoopController}. */
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedMetersPerSecond; 
        /** Minimum output of the driving PID. */
        public static final double kDrivingMinOutput = -1;
        /** maximum output of the driving PID. */
        public static final double kDrivingMaxOutput = 1;

        /** The P for the turning PID. */
        public static final double kTurningP = 1;
        /** The I for the turning PID. */
        public static final double kTurningI = 0;
        /** The D for the turning PID. */
        public static final double kTurningD = 0;
        /** The feedforward for the REV {@link ClosedLoopController}. */
        public static final double kTurningFF = 0;
        /** Minimum output of the turning PID. */
        public static final double kTurningMinOutput = -1;
        /** Minimum output of the turning PID. */
        public static final double kTurningMaxOutput = 1;

        // Inversion of drive motors
        // This will vary depending on how your wheels are oriented when you zero them.

        /** Whether or not to invert the front left drive {@link SparkMax}. */
        public static final boolean kLeftFrontInverted = true;
        /** Whether or not to invert the rear left drive {@link SparkMax}. */
        public static final boolean kLeftRearInverted = true;
        /** Whether or not to invert the front right drive {@link SparkMax}. */
        public static final boolean kRightFrontInverted = true;
        /** Whether or not to invert the rear right drive {@link SparkMax}. */
        public static final boolean kRightRearInverted = true;

        /** 
         * Whether or not to invert the turning {@link SparkMax} controllers.
         * Unless oriented differently, all of your turning motors should spin in the same direction.
         */
        public static final boolean kTurningMotorsInverted = true;

        /** 
         * Whether or not to invert the turning {@link CANcoder} encoders.
         * Unless oriented differently, all of your turning encoders should read values with the same sign.
         */
        public static final boolean kTurningEncoderInverted = false;

        /** The {@link IdleMode} of the driving motors. */
        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        /** The {@link IdleMode} of the turnings motors. */
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        /** The current limit of the driving motors in amps. */
        public static final int kDrivingMotorCurrentLimit = 35; // amps
        /** The current limit of the turning motors in amps. */
        public static final int kTurningMotorCurrentLimit = 35; // amps
    }

    /**
     * Describe how the robot should move to certain position and angle setpoints.
     */
    public static final class HeadingConstants {
        private HeadingConstants() {}

        /** Whether or not to reverse the gyro to make it CCW positive. */
        public static final boolean kGyroReversed = true;

        // This is used for making the robot face a certain direction

        /** The P for the PID making the robot rotate to certain angles. */
        public static final double kHeadingP = 0.025;
        /** The I for the PID making the robot rotate to certain angles. */
        public static final double kHeadingI = 0;
        /** The D for the PID making the robot rotate to certain angles. */
        public static final double kHeadingD = 0.001;
        /** The maximum output of the PID making the robot rotate to certain angles. */
        public static final double kHeadingMaxOutput = 0.8; // Percent
        /** The acceptable error in angle to the desired angle. */
        public static final double kHeadingTolerance = 1; // Degrees

        /** The P for the PID making the robot move to certain positions. */
        public static final double kTranslationP = 5;
        /** The I for the PID making the robot move to certain positions. */
        public static final double kTranslationI = 0;
        /** The D for the PID making the robot move to certain positions. */
        public static final double kTranslationD = 0;
        /** The maximum output of the PID making the robot move to certain positions. */
        public static final double kTranslationMaxOutput = 1; // Percent 
        /** The acceptable error in position to the desired position. */
        public static final double kTranslationTolerance = Units.inchesToMeters(3); // Meters
    }

    /**
     * Define ports and deadbands for the different controllers.
     */
    public static final class OIConstants {
        private OIConstants() {}

        /** The port of the driver controller. */
        public static final int kDriverControllerPort = 0;
        /** The port of the coDriver controller. */
        public static final int kCoDriverControllerPort = 1;
        
        /** The deadband of the driver controller. */
        public static final double kJoystickDeadband = 0.05;
        /** The deadband of the coDriver controller. */
        public static final double kTriggerDeadband = 0.5;
    }

    /**
     * Describe information about the game field.
     */
    public static final class FieldConstants {
        private FieldConstants() {}

        /** X axis: long side */
        public static final double kFieldWidthMeters = 17.5133;
        /** Y axis: short side */
        public static final double kFieldHeightMeters = 8.0518;

        /** Auto align positions. */
        public static final Pose2d[] kAutoAlignPositions = {
            /* Coral human player station positions */
            // Column 1
            new Pose2d(3.198, 4.191, new Rotation2d()),
            new Pose2d(3.198, 3.86, new Rotation2d()),
            // Column 2
            new Pose2d(3.685, 5.07, Rotation2d.fromDegrees(-60)),
            new Pose2d(3.685, 2.98, Rotation2d.fromDegrees(60)),
            // Column 3
            new Pose2d(3.968, 5.24, Rotation2d.fromDegrees(-60)),
            new Pose2d(3.968, 2.81, Rotation2d.fromDegrees(60)),
            // Column 4
            new Pose2d(5, 5.24, Rotation2d.fromDegrees(-120)),
            new Pose2d(5, 2.81, Rotation2d.fromDegrees(120)),
            // Column 5
            new Pose2d(5.29, 5.07, Rotation2d.fromDegrees(-60)),
            new Pose2d(5.29, 2.98, Rotation2d.fromDegrees(60)),
            // Column 6
            new Pose2d(5.8, 4.191, Rotation2d.fromDegrees(180)),
            new Pose2d(5.8, 3.86, Rotation2d.fromDegrees(180)),

            // Processor position
            new Pose2d(5.985, 0.51, Rotation2d.fromDegrees(90)),

        };

        /** Reef position for orbital controls. */
        public static final Translation2d kReefPosition = new Translation2d(4.49, 4.0255);
    }

    /**
     * Define how PathPlanner should move the robot during the autonomous period.
     */
    public static final class AutonConstants {
        private AutonConstants() {}

        /** The maximum speed PathPlanner will drive the robot in meters per second. */
        public static final double kAutoMaxSpeedMetersPerSecond = 3;
        /** The maximum speed PathPlanner will turn the robot wheels in radians per second. */
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;

        /** A {@link PIDConstants} object that describes the PID constants PathPlanner should use move the robot. */
        public static final PIDConstants kAutoTranslationPID = new PIDConstants(
            HeadingConstants.kTranslationP, 
            HeadingConstants.kTranslationI, 
            HeadingConstants.kTranslationD
        );

        /** A {@link PIDConstants} object that describes the PID constants PathPlanner should use to turn the robot wheels. */
        public static final PIDConstants kAutoAngularPID = new PIDConstants(
            5, 
            0, 
            0
        );

        /** A {@link PathFollowingController} that tells PathPlanner how to follow paths. */
        public static final PPHolonomicDriveController kPathPlannerController = new PPHolonomicDriveController( 
            kAutoTranslationPID, // Translation PID constants
            kAutoAngularPID // Rotation PID constants
        );

        /** A {@link RobotConfig} that describes physical information about the robot for PathPlanner. */
        public static final RobotConfig kPathPlannerRobotConfig = new RobotConfig(
            RobotConstants.kRobotMassKG, // robot mass
            RobotConstants.kRobotMOI, // robot moment of inertia
            new ModuleConfig(
                ModuleConstants.kWheelRadiusMeters, // drive wheel radius
                ModuleConstants.kDriveWheelFreeSpeedMetersPerSecond, // drive motor maximum capable speed
                ModuleConstants.kWheelCOF, // wheel friction on carpet
                ModuleConstants.kMotorGearboxConfig, // config for NEO motor gearbox
                ModuleConstants.kDrivingMotorReduction, // gear ratio
                ModuleConstants.kDrivingMotorCurrentLimit, // drive motor current limit
                1 // number of drive motors per module
            ),
            DriveConstants.kDriveKinematics.getModules() // module offsets from center of chassis
        );
    }

    /** Describes physical constraints of the NEO motors. */
    public static final class NEOMotorConstants {
        private NEOMotorConstants() {}

        /** The maximum speed the motors go run at in revolutions per minute. */
        public static final double kFreeSpeedRpm = 5676;
        /** The maximum speed the motors go run at in revolutions per second. */
        public static final double kFreeSpeedRps = kFreeSpeedRpm / 60;
    }

    /**
     * Describe how the {@link ElevatorSubsystem} should move the elevator.
     */
    public static final class ElevatorConstants {
        private ElevatorConstants() {}

        /** CAN ID of the left elevator motor controller. */
        public static final int kElevatorLeftCanId = 35;
        /** CAN ID of the right elevator motor controller. */
        public static final int kElevatorRightCanId = 36;

        /** Channel of the elevator bottom limit switch. */
        public static final int kBottomSwitchChannel = 0;
        /** Channel of the elevator top limit switch. */
        public static final int kTopSwitchChannel = 1;

        /** Idle mode of the elevator motors. */
        public static final IdleMode kElevatorIdleMode = IdleMode.kBrake;

        /** Current limit of the elevator motors. */
        public static final int kElevatorCurrentLimit = 35;

        /** Whether to invert the left elevator motor. */
        public static final boolean kElevatorLeftInverted = false;
        /** Whether to invert the right elevator motor. */
        public static final boolean kElevatorRightInverted = false;

        /** The reduction in distance calculated by endcoders due to gear ratio. */
        public static final double kElevatorReduction = 40;
        /** The diameter of the gear/wheel that moves the elevator. */
        public static final double kGearDiameter = Units.inchesToMeters(1);
        /** The circumference of the gear/wheel that moves the elevator. */
        public static final double kGearCircumference = kGearDiameter * Math.PI;
        /** The conversion factor that converts from motor rotations to meters. */
        public static final double kElevatorEncoderPositionFactor = kGearCircumference / kElevatorReduction;
        /** The conversion factor that converts from motor rotations per minute to meters per second. */
        public static final double kElevatorEncoderVelocityFactor = (kGearCircumference / kElevatorReduction) / 60;

        /** The P for the elevator PID. */
        public static final double kElevatorP = 1;
        /** The I for the elevator PID. */
        public static final double kElevatorI = 0;
        /** The D for the elevator PID. */
        public static final double kElevatorD = 0;

        /** The S gain for the elevator feedforward. */
        public static final double kElevatorS = 0;
        /** The gravity gain for the elevator feedforward. */
        public static final double kElevatorG = 0;
        /** The V gain for the elevator feedforward. */
        public static final double kElevatorV = 0;
        /** The A gain for the elevator feedforward. */
        public static final double kElevatorA = 0;

        /** The maximum speed the elevator can move at with full power. */
        public static final double kElevatorFreeSpeedMetersPerSecond = NEOMotorConstants.kFreeSpeedRpm * kElevatorEncoderVelocityFactor;

        /** The maximum allowed speed the elavtor should move at. */
        public static final double kElevatorMaxSpeedMetersPerSecond = 0.1;

        /** The maximum allowed acceleration of the elevator. */
        public static final double kElevatorMaxAccelerationMetersPerSecondSquared = 0.1;

        /** The physical height of the elevator. */
        public static final double kPhysicalHeightLimit = 1;
        /** The preferences key for the elevator height. */
        public static final String kPhysicalHeightLimitKey = "physical_elevator_height";

        // TODO: add heights
        /** The heights, in meters, of every elevator level. */
        public static final double[] kElevatorLevelHeights = {};
    }

    /**
     * Describe how the {@link AlgaeIntakeRollerSubsystem} should intake and shoot algae.
     */
    public static final class AlgaeIntakeConstants {
        private AlgaeIntakeConstants() {}

        /** CAN ID of the algae intake roller motor. */
        public static final int kAlgaeIntakeRollerMotorCanId = 33;
        /** CAN ID of the algae intake arm motor. */
        public static final int kAlgaeIntakeArmMotorCanId = 34;

        /** Whether or not to invert the algae intake roller motor. */
        public static final boolean kAlgaeIntakeRollerInverted = false;
        /** Whether or not to invert the algae intake arm motor. */
        public static final boolean kAlgaeIntakeArmInverted = false;

        /** The {@link IdleMode} of alage intake roller. */
        public static final IdleMode kAlgaeIntakeRollerIdleMode = IdleMode.kBrake;
        /** The {@link IdleMode} of alage intake arm. */
        public static final IdleMode kAlgaeIntakeArmIdleMode = IdleMode.kBrake;

        /** The current limit of the alage intake roller in amps. */
        public static final int kAlageIntakeRollerSmartCurrentLimit = 35;
        /** The current limit of the alage intake arm in amps. */
        public static final int kAlageIntakeArmSmartCurrentLimit = 35;
        
        /** The speed the roller should be run at when intaking and shooting. */
        public static final double kRollerSpeed = 0.1;

        /** The speed the arm should be run at when movin in and out. */
        public static final double kArmSpeed = 0.1;
        /** How long the arm motor should be run until it is garunteed to be in the correct position. */
        public static final double kArmTimeout = 1;
        /** The acceptable error for what is considered fully in or out. */
        public static final double kArmErrorThreshold = 0.05;
    }

    /** Describe how the {@link CoralIntakeSubsystem} should intake and shoot coral. */
    public static final class CoralIntakeConstants {
        private CoralIntakeConstants() {}

        /** The CAN ID for the intake motor. */
        public static final int kCoralIntakeMotorCanId = 32;
        /** Whether the coral intake is inverted */
        public static final boolean kCoralIntakeInverted = false;
        /** The idle mode of the coral intake motor. */
        public static final IdleMode kCoralIntakeIdleMode = IdleMode.kBrake;
        /** The current limit for the coral intake motor. */
        public static final int kSmartCurrentLimit = 35;
        
        /** The speed the coral intake should be run at when intaking or shooting. */
        public static final double kCoralIntakeSpeed = 0.1;

        /**The channel for the digital input */
        public static final int kSensorChannel = 1;
    }

    /**
     * Describe how the {@link ArmSubsystem} should rotate the coral arm.
     */
    public static final class ArmConstants {
        private ArmConstants() {}

        /** The CAN ID of the arm motor. */
        public static final int kArmMotorCanId = 30;
        /** The channel ID for the motor encoder. */
        public static final int kArmEncoderId = 31;
        /** The smart current limit for the motor */
        public static final int kSmartCurrentLimit = 35;
        /** The idle mode of the motor. */
        public static final IdleMode kIdleMode = IdleMode.kBrake;
        /** Whether to invert the direction of the arm motor. */
        public static final boolean kArmMotorInvered = false;
        /** Whether to invert the direction of the encoder. */
        public static final boolean kEncoderInverted = false;

        /** The maximum speed of the arm in rotations per second. */
        public static final double kArmMaxSpeedRPS = 0.1;
        /** The maximum acceleration of the arm in rotations per second squared. */
        public static final double kArmMaxAccelerationRPSSquared = 0.1;

        /** The period of the duty cycle for the arm. */
        public static final int kArmDutyCyclePeriod = 4096;
        /** The angle offset for the motor encoder such that when the encoder returns 0 the arm is parallel to the floor. */
        public static final Rotation2d kEncoderAngleOffset = Rotation2d.fromDegrees(0);

        /** The P for the arm PID controller. */
        public static final double kArmP = 1;
        /** The I for the arm PID controller. */
        public static final double kArmI = 0;
        /** The D for the arm PID controller. */
        public static final double kArmD = 0;
        
        /** The S gain for the arm feedforward. */
        public static final double kArmS = 0;
        /** The gravity gain for the arm feedforward. */
        public static final double kArmG = 0;
        /** The V gain for the arm feedforward. */
        public static final double kArmV = 0;
        /** The A gain for the arm feedforward. */
        public static final double kArmA = 0;

        // TODO: add angles
        /** The angles of the arm for each elevator level. */
        public static final Rotation2d[] kArmLevelAngles = {};
    }

    /**
     * Describe how the {@link PhotonSubsystem} should interface with Photon Vision.
     */
    public static final class PhotonConstants {
        private PhotonConstants() {}

        // TODO: set names and transformations
        /** The name of the Photon Vision cameras. */
        public static final String[] kCameraNames = { "jojo bizar" };
        /** The transformations that describe how to move from the center of the robot to the Photon Vision cameras. */
        public static final Transform3d[] kRobotToCameraTransformations = {
            new Transform3d(
                new Translation3d(0,0,0),
                new Rotation3d(0,0,0)
            )
        };

        /** The april tage layout Photon Vision should use. */
        public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        /** How Photon Vision should use april tag data to determine position. */
        public static final PoseStrategy kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    }

    /**
     * Used by {@link LoggingUtils} to log data from the PDH.
     */
    public static final class PDHConstants {
        private PDHConstants() {}

        /** The CAN ID of the PDH */
        public static final int kPDHCanID = 10;
    }
}
