// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.FieldConstants;
// import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.coral.GoToLevel;
import frc.robot.commands.coral.SetElevatorVelocity;
import frc.robot.commands.drive.RobotGotoAngle;
// import frc.robot.commands.drive.AutoAlign;
// import frc.robot.commands.drive.RobotOrbitPoint;
import frc.robot.commands.drive.TurningMotorsTest;
// import frc.robot.subsystems.AlgaeIntakeArmSubsystem;
// import frc.robot.subsystems.AlgaeIntakeRollerSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
// import frc.robot.utils.FieldUtils;
import frc.robot.utils.InputMappings;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    /** The {@link VisionSubsystem} of the robot. */
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    /** The {@link DriveSubsystem} of the robot. */
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_visionSubsystem);
    /** The {@link ElevatorSubsystem} of the robot. */
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    /** The {@link ArmSubsystem} of the robot. */
    private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
    // /** The {@link AlgaeIntakeRollerSubsystem} of the robot. */
    // private final AlgaeIntakeRollerSubsystem m_algaeIntakeRollerSubsystem = new AlgaeIntakeRollerSubsystem();
    // /** The {@link AlgaeIntakeArmSubsystem} of the robot. */
    // private final AlgaeIntakeArmSubsystem m_algaeIntakeArmSubsystem = new AlgaeIntakeArmSubsystem();
    // /** The {@link CoralIntakeSubsystem} of the robot. */
    private final CoralIntakeSubsystem m_coralIntakeSubsystem = new CoralIntakeSubsystem();

    // Controllers
    /** The {@link CommandXboxController} object that represents the driver controller. */
    private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    /** The {@link CommandXboxController} object that represents the coDriver controller. */
    private final CommandXboxController m_coDriverController = new CommandXboxController(OIConstants.kCoDriverControllerPort);

    // Sendable choosers to dictate what the robot does during auton
    /** The {@link SendableChooser} sent to Elastic for the auton path to follow. */
    private SendableChooser<Command> m_autonAction;

    /**
     * The container for the robot. Contains <code>Subsystem</code> objects, OI devices, and <code>Command</code> objects.
     */
    public RobotContainer() {
        InputMappings.registerController("driver", m_driverController);
        InputMappings.registerController("coDriver", m_coDriverController);

        InputMappings.addChoosers(Shuffleboard.getTab("Input"));

        // Configure the button bindings
        configureButtonBindings();

        // Configure named commands for pathplanner
        configureNamedCommands();

        m_autonAction = AutoBuilder.buildAutoChooser();

        // Configure default commands
        m_driveSubsystem.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_driveSubsystem.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kJoystickDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kJoystickDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kJoystickDeadband),
                    true, true
                ), m_driveSubsystem
            )
        );
        
        // Adding options to the sendable choosers
        // applyCommands(m_autonFirstAction);

        // Put choosers on the dashboard
        Shuffleboard.getTab("Autonomous").add("First Action", m_autonAction).withSize(2, 1);

        // DEBUG: widgets for testing swerve modules
        Shuffleboard.getTab("Swerve").add("Module Drive Test", new RunCommand(
            () -> m_driveSubsystem.drive(
                0.03,
                0,
                0,
                false, true),
                m_driveSubsystem));
        Shuffleboard.getTab("Swerve").add("Module Turn Test", new TurningMotorsTest(m_driveSubsystem));
    }

    /**
     * Use this method to define your button -> <code>Command</code> mappings.
     */
    private void configureButtonBindings() {
        //------------------------------------------- Driver buttons -------------------------------------------

        // m_driverController.rightBumper()
        //     .onTrue(new InstantCommand((() -> m_driveSubsystem.setHeading(0))));
        // InputMappings.event("driver", "autoAlign")
        //     .whileTrue(new AutoAlign(driveSubsystem));

        // InputMappings.event("driver", "orbitReef")
        //     .whileTrue(new RobotOrbitPoint(driveSubsystem,
        //         () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kJoystickDeadband),
        //         () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kJoystickDeadband),
        //         FieldConstants.kReefPosition)
        //     );

        InputMappings.event("driver", "leftHuman")
            .onTrue(new RobotGotoAngle(
                m_driveSubsystem,
                FieldConstants.kHumanPlayerStationAngle.unaryMinus(),
                true,
                () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kJoystickDeadband),
                () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kJoystickDeadband),
                () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kJoystickDeadband)
            ));

        InputMappings.event("driver", "rightHuman")
            .onTrue(new RobotGotoAngle(
                m_driveSubsystem,
                FieldConstants.kHumanPlayerStationAngle,
                true,
                () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kJoystickDeadband),
                () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kJoystickDeadband),
                () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kJoystickDeadband)
            ));

        //------------------------------------------- coDriver buttons -------------------------------------------

        // InputMappings.event("coDriver", "algaeIntake")
        //     .onTrue(m_algaeIntakeRollerSubsystem.intakeAlgae());
        // InputMappings.event("coDriver", "algaeShoot")
        //     .onTrue(m_algaeIntakeRollerSubsystem.shootAlgae());
        // InputMappings.event("coDriver", "toggleAlgaeIntakeArm")
        //     .onTrue(m_algaeIntakeArmSubsystem.armToggle());

        InputMappings.event("coDriver", "coralIntake")    
            .whileTrue(m_coralIntakeSubsystem.intakeCoral(true));

        InputMappings.event("coDriver", "coralShoot")
            .whileTrue(m_coralIntakeSubsystem.shootCoral());

        InputMappings.event("coDriver", "coralReverse")
            .whileTrue(m_coralIntakeSubsystem.reverseCoral());

        InputMappings.event("coDriver", "elevatorUp")
        // m_coDriverController.povUp()
            .onTrue(new SetElevatorVelocity(m_armSubsystem, m_elevatorSubsystem, 0.8))
            .onFalse(new InstantCommand(m_elevatorSubsystem::stop));
        InputMappings.event("coDriver", "elevatorDown")
        // m_coDriverController.povDown()
        .onTrue(new SetElevatorVelocity(m_armSubsystem, m_elevatorSubsystem, -0.8))
            .onFalse(new InstantCommand(() -> m_elevatorSubsystem.setElevatorVelocity(0), m_elevatorSubsystem));

        InputMappings.event("coDriver", "armUp")
        // m_coDriverController.povLeft()
            .onTrue(m_armSubsystem.goToAngle(Rotation2d.kZero, false));
        InputMappings.event("coDriver", "armDown")
        // m_coDriverController.povRight()
            .onTrue(m_armSubsystem.goToAngle(Rotation2d.fromDegrees(32.4), false));

        m_coDriverController.leftBumper()
            .onTrue(m_armSubsystem.goToAngle(Rotation2d.fromDegrees(90), false));
        
        InputMappings.event("coDriver", "coralL1")
        // m_coDriverController.a()
            .onTrue(new GoToLevel(m_armSubsystem, m_elevatorSubsystem, 0));
        InputMappings.event("coDriver", "coralL2")
        // m_coDriverController.b()
            .onTrue(new GoToLevel(m_armSubsystem, m_elevatorSubsystem, 1));
        InputMappings.event("coDriver", "coralL3")
        // m_coDriverController.x()
            .onTrue(new GoToLevel(m_armSubsystem, m_elevatorSubsystem, 2));
        // m_coDriverController.y()4\
    
        InputMappings.event("coDriver", "coralL4")
            .onTrue(new GoToLevel(m_armSubsystem, m_elevatorSubsystem, 3));
    }

    /**
     * Configures the {@link NamedCommands} for PathPlanner.
     */
    private void configureNamedCommands() {
        // NamedCommands.registerCommand("algaeIntake", m_algaeIntakeRollerSubsystem.intakeAlgae().withTimeout(2));
        // NamedCommands.registerCommand("algaeShoot", m_algaeIntakeRollerSubsystem.shootAlgae().withTimeout(2));
        // NamedCommands.registerCommand("algaeIntakeArmIn", m_algaeIntakeArmSubsystem.armIn());
        // NamedCommands.registerCommand("algaeIntakeArmOut", m_algaeIntakeArmSubsystem.armOut());

        NamedCommands.registerCommand("coralIntake", m_coralIntakeSubsystem.intakeCoral(true));
        NamedCommands.registerCommand("shootCoral", m_coralIntakeSubsystem.shootCoral().withTimeout(0.5));

        NamedCommands.registerCommand("coralL1", new GoToLevel(m_armSubsystem, m_elevatorSubsystem, 0));
        NamedCommands.registerCommand("coralL2", new GoToLevel(m_armSubsystem, m_elevatorSubsystem, 1));
        NamedCommands.registerCommand("coralL3", new GoToLevel(m_armSubsystem, m_elevatorSubsystem, 2));
        NamedCommands.registerCommand("coralL4", new GoToLevel(m_armSubsystem, m_elevatorSubsystem, 3));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return The command to run in autonomous.
     */
    public Command getAutonomousCommand() {
        return m_autonAction.getSelected();
    }

    /**
     * Gets the {@link DriveSubsystem} of the container.
     * @return The <code>DriveSubsystem</code>.
     */
    public DriveSubsystem getDriveSubsystem() {
        return m_driveSubsystem;
    }
}
