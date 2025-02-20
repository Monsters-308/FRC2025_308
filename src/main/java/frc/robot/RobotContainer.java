// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drive.AutoAlign;
import frc.robot.commands.drive.RobotOrbitPoint;
import frc.robot.commands.drive.TurningMotorsTest;
// import frc.robot.subsystems.AlgaeIntakeArmSubsystem;
// import frc.robot.subsystems.AlgaeIntakeRollerSubsystem;
// import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    /** The {@link PhotonSubsystem} of the robot. */
    private final PhotonSubsystem m_photonSubsystem = new PhotonSubsystem();
    /** The {@link DriveSubsystem} of the robot. */
    public final DriveSubsystem driveSubsystem = new DriveSubsystem(
        m_photonSubsystem::getEstimatedGlobalPose
    );

    // /** The {@link ElevatorSubsystem} of the robot. */
    // private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    // /** The {@link AlgaeIntakeRollerSubsystem} of the robot. */
    // private final AlgaeIntakeRollerSubsystem m_algaeIntakeRollerSubsystem = new AlgaeIntakeRollerSubsystem();
    // /** The {@link AlgaeIntakeArmSubsystem} of the robot. */
    // private final AlgaeIntakeArmSubsystem m_algaeIntakeArmSubsystem = new AlgaeIntakeArmSubsystem();
    // /** The {@link CoralIntakeSubsystem} of the robot. */
    // private final CoralIntakeSubsystem m_coralIntakeSubsystem = new CoralIntakeSubsystem();

    // Controllers
    /** The {@link CommandXboxContoller} object that represents the driver controller. */
    final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    /** The {@link CommandXboxContoller} object that represents the coDriver controller. */
    final CommandXboxController m_coDriverController = new CommandXboxController(OIConstants.kCoDriverControllerPort);

    // Sendable choosers to dictate what the robot does during auton
    /** The {@link SendableChooser} send to Elastic for the first auton path to follow. */
    SendableChooser<Command> m_autonFirstAction = new SendableChooser<>();
    /** The {@link SendableChooser} send to Elastic for the second auton path to follow. */
    SendableChooser<Command> m_autonSecondAction = new SendableChooser<>();


    /**
     * The container for the robot. Contains <code>Subsystem</code> objects, OI devices, and <code>Command</code> objects.
     */
    public RobotContainer() {
        InputMappings.registerController("driver", m_driverController);
        InputMappings.registerController("coDriver", m_coDriverController);

        InputMappings.addChoosers(Shuffleboard.getTab("Input"));

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> driveSubsystem.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kJoystickDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kJoystickDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kJoystickDeadband),
                    true, true),
                driveSubsystem));
        
        // Adding options to the sendable choosers
        applyCommands(m_autonFirstAction);
        applyCommands(m_autonSecondAction);

        // Put choosers on the dashboard
        Shuffleboard.getTab("Autonomous").add("First Action", m_autonFirstAction).withSize(2, 1);
        Shuffleboard.getTab("Autonomous").add("Second Action", m_autonSecondAction).withSize(2, 1);

        // DEBUG: widgets for testing swerve modules
        Shuffleboard.getTab("Swerve").add("Module Drive Test", new RunCommand(
            () -> driveSubsystem.drive(
                0.03,
                0,
                0,
                false, true),
                driveSubsystem));
        Shuffleboard.getTab("Swerve").add("Module Turn Test", new TurningMotorsTest(driveSubsystem));

        // FAILSAFE: widgets for manually setting robot position if the limelight is not working or can't view the april tags.
        Shuffleboard.getTab("Autonomous").add("Set Amp Side",
            new InstantCommand(() -> driveSubsystem.resetOdometry(FieldUtils.flipRed(
                new Pose2d(
                    0.73, 
                    6.73, 
                    Rotation2d.fromDegrees(-120))
            )))
            .ignoringDisable(true)
        );

        Shuffleboard.getTab("Autonomous").add("Set Middle",
            new InstantCommand(() -> driveSubsystem.resetOdometry(FieldUtils.flipRed(
                new Pose2d(
                    1.5,
                    5.55,
                    Rotation2d.fromDegrees(180))
            )))
            .ignoringDisable(true)
        );

        Shuffleboard.getTab("Autonomous").add("Set Source Side",
            new InstantCommand(() -> driveSubsystem.resetOdometry(FieldUtils.flipRed(
                new Pose2d(
                    0.73, 
                    4.39, 
                    Rotation2d.fromDegrees(120))
            )))
            .ignoringDisable(true)
        );

    }

    /**
     * Use this method to define your button -> <code>Command</code> mappings.
     */
    private void configureButtonBindings() {
        //------------------------------------------- Driver buttons -------------------------------------------

        InputMappings.event("driver", "autoAlign")
            .onTrue(new AutoAlign(driveSubsystem));

        InputMappings.event("driver", "orbitReef")
            .whileTrue(new RobotOrbitPoint(driveSubsystem,
                () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kJoystickDeadband),
                () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kJoystickDeadband),
                FieldConstants.kReefPosition)
            );

        //------------------------------------------- coDriver buttons -------------------------------------------

        // InputMappings.event("coDriver", "algaeIntake")
        //     .onTrue(m_algaeIntakeRollerSubsystem.intakeAlgae());
        // InputMappings.event("coDriver", "algaeShoot")
        //     .onTrue(m_algaeIntakeRollerSubsystem.shootAlgae());
        // InputMappings.event("coDriver", "toggleAlgaeIntakeArm")
        //     .onTrue(m_algaeIntakeArmSubsystem.armToggle());

        // InputMappings.event("coDriver", "coralIntake")
        //     .whileTrue(m_coralIntakeSubsystem.intakeCoral(true));
        // InputMappings.event("coDriver", "coralShoot")
        //     .whileTrue(m_coralIntakeSubsystem.shootCoral());

        // InputMappings.event("coDriver", "elevator1")
        //     .onTrue(m_elevatorSubsystem.goToLevel(0, true));
        // InputMappings.event("coDriver", "elevator2")
        //     .onTrue(m_elevatorSubsystem.goToLevel(1, true));
        // InputMappings.event("coDriver", "elevator3")
        //     .onTrue(m_elevatorSubsystem.goToLevel(2, true));
        // InputMappings.event("coDriver", "elevator4")
        //     .onTrue(m_elevatorSubsystem.goToLevel(3, true));
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
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            m_autonFirstAction.getSelected(),
            m_autonSecondAction.getSelected()
        );
    }
}
