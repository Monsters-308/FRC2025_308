// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drive.RobotFacePoint;
import frc.robot.commands.drive.RobotGotoAngle;
import frc.robot.commands.drive.TurningMotorsTest;
import frc.robot.commands.vision.DefaultLimelightPipeline;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.FieldUtils;
import frc.utils.InputMappings;

/*
 * This class is where the bulk of the robot should be declared.    Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).    Instead, the structure of the robot
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

    // Controllers
    final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
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
            new RobotFacePoint(m_visionSubsystem, m_driveSubsystem, () -> 0, () -> 0, FieldConstants.kRandomPosition)
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
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {


        //------------------------------------------- Driver buttons -------------------------------------------


        // Right bumper: puts drive into x mode
        new JoystickButton(m_driverController, Button.kRightBumper.value)
            .whileTrue(new RunCommand(
                m_driveSubsystem::setX,
                m_driveSubsystem));
        
        // Left bumper: sets gyro to 0 degrees
        new JoystickButton(m_driverController, Button.kLeftBumper.value)
            .onTrue(new InstantCommand(
                m_driveSubsystem::zeroHeading));

        // A button: makes robot face a point in space
        new JoystickButton(m_driverController, Button.kA.value)
            .whileTrue(
                new RobotFacePoint(
                    m_visionSubsystem, 
                    m_driveSubsystem, 
                    m_driverController::getLeftY,
                    m_driverController::getLeftX,
                    FieldConstants.kRandomPosition)
                );
        
        // Dpad up: makes robot face 0 degrees
        new POVButton(m_driverController, 0)
            .toggleOnTrue(
                new RobotGotoAngle(
                    m_driveSubsystem,
                    0,
                    false,
                    m_driverController::getLeftY,
                    m_driverController::getLeftX,
                    m_driverController::getRightX));

        // Dpad right: makes robot face 90 degrees to the right
        new POVButton(m_driverController, 90)
            .toggleOnTrue(
                new RobotGotoAngle(
                    m_driveSubsystem,
                    -90,
                    false,
                    m_driverController::getLeftY,
                    m_driverController::getLeftX,
                    m_driverController::getRightX));

        // Dpad down: makes robot face 180 degrees
        new POVButton(m_driverController, 180)
            .toggleOnTrue(
                new RobotGotoAngle(
                    m_driveSubsystem,
                    180,
                    false,
                    m_driverController::getLeftY,
                    m_driverController::getLeftX,
                    m_driverController::getRightX));
                                
        // Dpad left: makes robot face 90 degrees to the left
        new POVButton(m_driverController, 270)
            .toggleOnTrue(
                new RobotGotoAngle(
                    m_driveSubsystem,
                    90,
                    false,
                    m_driverController::getLeftY,
                    m_driverController::getLeftX,
                    m_driverController::getRightX));


        //------------------------------------------- coDriver buttons -------------------------------------------
        

        // (Put codriver controls here)
        InputMappings.registerController("codriver", m_coDriverController);
    }

    /**
     * Function for adding all of our auton paths to each of the choosers
     * @param autonChooser The sendable chooser being used for auton.
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
