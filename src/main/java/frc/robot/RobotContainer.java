// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drive.AutoAlign;
import frc.robot.commands.drive.RobotFacePoint;
import frc.robot.commands.drive.RobotOrbitPoint;
import frc.robot.commands.drive.TurningMotorsTest;
import frc.robot.commands.vision.DefaultLimelightPipeline;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.FieldUtils;
import frc.utils.InputMappings;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    private final PhotonSubsystem m_photonSubsystem = new PhotonSubsystem();
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(
        m_visionSubsystem::getRobotPosition,
        m_visionSubsystem::getTimeStampEstimator,
        m_photonSubsystem::getEstimatedGlobalPose
    );

    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final AlgaeIntakeSubsystem m_algaeIntakeSubsystem = new AlgaeIntakeSubsystem();

    // Controllers
    final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    final CommandXboxController m_coDriverController = new CommandXboxController(OIConstants.kCoDriverControllerPort);

    // Sendable choosers to dictate what the robot does during auton
    SendableChooser<Command> m_autonFirstAction = new SendableChooser<>();
    SendableChooser<Command> m_autonSecondAction = new SendableChooser<>();


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure limelight default pipeline
        m_visionSubsystem.setDefaultCommand(new DefaultLimelightPipeline(m_visionSubsystem));

        // Configure default commands
        m_driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_driveSubsystem.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kJoystickDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kJoystickDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kJoystickDeadband),
                    true, true),
                m_driveSubsystem));

            
        // "registerCommand" lets pathplanner identify our commands so we can use them in pathplanner autons
        // Here's RobotFacePoint as an example:
        NamedCommands.registerCommand("FacePoint",
            new RobotFacePoint(m_visionSubsystem, m_driveSubsystem, () -> 0, () -> 0, FieldConstants.kRandomPosition, false)
        );
        
        // Adding options to the sendable choosers
        applyCommands(m_autonFirstAction);
        applyCommands(m_autonSecondAction);

        // Put choosers on the dashboard
        Shuffleboard.getTab("Autonomous").add("First Action", m_autonFirstAction).withSize(2, 1);
        Shuffleboard.getTab("Autonomous").add("Second Action", m_autonSecondAction).withSize(2, 1);

        // DEBUG: widgets for testing swerve modules
        Shuffleboard.getTab("Swerve").add("Module Drive Test", new RunCommand(
            () -> m_driveSubsystem.drive(
                0.03,
                0,
                0,
                false, true),
                m_driveSubsystem));
        Shuffleboard.getTab("Swerve").add("Module Turn Test", new TurningMotorsTest(m_driveSubsystem));

        // FAILSAFE: widgets for manually setting robot position if the limelight is not working or can't view the april tags.
        Shuffleboard.getTab("Autonomous").add("Set Amp Side",
            new InstantCommand(() -> m_driveSubsystem.resetOdometry(FieldUtils.flipRed(
                new Pose2d(
                    0.73, 
                    6.73, 
                    Rotation2d.fromDegrees(-120))
            )))
            .ignoringDisable(true)
        );

        Shuffleboard.getTab("Autonomous").add("Set Middle",
            new InstantCommand(() -> m_driveSubsystem.resetOdometry(FieldUtils.flipRed(
                new Pose2d(
                    1.5,
                    5.55,
                    Rotation2d.fromDegrees(180))
            )))
            .ignoringDisable(true)
        );

        Shuffleboard.getTab("Autonomous").add("Set Source Side",
            new InstantCommand(() -> m_driveSubsystem.resetOdometry(FieldUtils.flipRed(
                new Pose2d(
                    0.73, 
                    4.39, 
                    Rotation2d.fromDegrees(120))
            )))
            .ignoringDisable(true)
        );

    }

    /**
     * Use this method to define your button->command mappings.
     */
    private void configureButtonBindings() {
        //------------------------------------------- Driver buttons -------------------------------------------
        InputMappings.registerController("driver", m_driverController);

        InputMappings.event("driver", "autoAlign")
            .onTrue(new AutoAlign(m_driveSubsystem));

        InputMappings.event("driver", "orbitReef")
            .whileTrue(new RobotOrbitPoint(m_driveSubsystem,
                () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kJoystickDeadband),
                () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kJoystickDeadband),
                FieldConstants.kReefPosition)
            );

        //------------------------------------------- coDriver buttons -------------------------------------------
        InputMappings.registerController("coDriver", m_coDriverController);

        InputMappings.event("coDriver", "alageIntake")
            .onTrue(m_algaeIntakeSubsystem.intakeAlgae());
        InputMappings.event("coDriver", "alageShoot")
            .onTrue(m_algaeIntakeSubsystem.shootAlgae());
        InputMappings.event("coDriver", "toggleAlgaeIntakeArm")
            .onTrue(m_algaeIntakeSubsystem.armToggle());

        InputMappings.event("coDriver", "elevator1")
            .onTrue(m_elevatorSubsystem.goToLevel(0, true));
        InputMappings.event("coDriver", "elevator2")
            .onTrue(m_elevatorSubsystem.goToLevel(0, true));
        InputMappings.event("coDriver", "elevator3")
            .onTrue(m_elevatorSubsystem.goToLevel(0, true));
        InputMappings.event("coDriver", "elevator4")
            .onTrue(m_elevatorSubsystem.goToLevel(0, true));
    }

    /**
     * Function for adding all of our auton paths to each of the choosers
     * @param autonChooser The {@link SendableChooser} being used for auton.
     */
    private void applyCommands(SendableChooser<Command> autonChooser){
        autonChooser.setDefaultOption("Do Nothing", new WaitCommand(15));
        autonChooser.addOption("Move One Meter", new PathPlannerAuto("Move One Meter"));
        autonChooser.addOption("Two Meter Spin", new PathPlannerAuto("Two Meter Spin"));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            m_autonFirstAction.getSelected(),
            m_autonSecondAction.getSelected()
        );
    }
}
