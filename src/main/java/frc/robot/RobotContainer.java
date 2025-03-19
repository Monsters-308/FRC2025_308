// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashSet;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.coral.GoToLevel;
import frc.robot.commands.coral.IntakeCoral;
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
import frc.robot.utils.InputMappings;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
    /** The {@link SendableChooser} containing the auto to use during auton if set manually. */
    public final SendableChooser<Command> m_manualAuton;
    /** The {@link SendableChooser} containing the auton start position. Can be left, center, or right. */
    private final SendableChooser<String> m_autonStartPose = new SendableChooser<>();
    /** The {@link SendableChooser} containing the number of corals to attempt to score during auton. */
    private final SendableChooser<String> m_autonNumCorals = new SendableChooser<>();
    /** The {@link SendableChooser} containing the level to attempt to score the first coral to during auton. */
    private final SendableChooser<Integer> m_autonFirstCoralLevel = new SendableChooser<>();
    /** The {@link SendableChooser} containing the level to attempt to score the second coral to during auton. */
    private final SendableChooser<Integer> m_autonSecondCoralLevel = new SendableChooser<>();

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

        // Configure default commands
        m_driveSubsystem.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_driveSubsystem.drive(
                    0.3 * -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kJoystickDeadband),
                    0.3 * -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kJoystickDeadband),
                    0.3 * -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kJoystickDeadband),
                    true, true
                ), m_driveSubsystem
            )
        );

        // Put choosers on the dashboard
        ShuffleboardTab autonTab = Shuffleboard.getTab("Autonomous");

        m_manualAuton = AutoBuilder.buildAutoChooser();
        m_manualAuton.setDefaultOption("None", null);

        autonTab.add("Manual", m_manualAuton);

        m_autonStartPose.addOption("Left", "Left");
        m_autonStartPose.setDefaultOption("Center", "Center");
        m_autonStartPose.addOption("Right", "Right");

        autonTab.add("Start Position", m_autonStartPose);

        m_autonNumCorals.setDefaultOption("One", "One Coral");
        m_autonNumCorals.addOption("Two", "Two Corals");
        
        autonTab.add("Number of Corals", m_autonNumCorals);

        m_autonFirstCoralLevel.setDefaultOption("Level #1", 0);
        m_autonFirstCoralLevel.addOption("Level #2", 1);
        m_autonFirstCoralLevel.addOption("Level #3", 2);
        m_autonFirstCoralLevel.addOption("Level #4", 3);
        
        autonTab.add("1st Coral Level", m_autonFirstCoralLevel);

        m_autonSecondCoralLevel.setDefaultOption("Level #1", 0);
        m_autonSecondCoralLevel.addOption("Level #2", 1);
        m_autonSecondCoralLevel.addOption("Level #3", 2);
        m_autonSecondCoralLevel.addOption("Level #4", 3);
        
        autonTab.add("2nd Coral Level", m_autonSecondCoralLevel);

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
        
        // Button for reseting field relative controls
        InputMappings.event("driver", "resetFieldRelative")
            .onTrue(new InstantCommand(m_driveSubsystem::resetFieldRelative));
    
        // InputMappings.event("driver", "autoAlign")
        //     .whileTrue(new AutoAlign(driveSubsystem));

        // InputMappings.event("driver", "orbitReef")
        //     .whileTrue(new RobotOrbitPoint(driveSubsystem,
        //         () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kJoystickDeadband),
        //         () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kJoystickDeadband),
        //         FieldConstants.kReefPosition)
        //     );

        InputMappings.event("driver", "leftHuman")
            .whileTrue(new RobotGotoAngle(
                m_driveSubsystem,
                FieldConstants.kHumanPlayerStationAngle.unaryMinus(),
                () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kJoystickDeadband),
                () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kJoystickDeadband),
                () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kJoystickDeadband)
            ));

        InputMappings.event("driver", "rightHuman")
            .whileTrue(new RobotGotoAngle(
                m_driveSubsystem,
                FieldConstants.kHumanPlayerStationAngle,
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
            .whileTrue(new IntakeCoral(m_armSubsystem, m_coralIntakeSubsystem));

        InputMappings.event("coDriver", "coralShoot")
            .whileTrue(m_coralIntakeSubsystem.shootCoral());

        InputMappings.event("coDriver", "coralReverse")
            .whileTrue(m_coralIntakeSubsystem.reverseCoral());

        InputMappings.event("coDriver", "elevatorUp")
            .onTrue(m_elevatorSubsystem.goToVelocity(0.4))
            .onFalse(new InstantCommand(m_elevatorSubsystem::stop));

        InputMappings.event("coDriver", "elevatorDown")
            .onTrue(m_elevatorSubsystem.goToVelocity(-0.4))
            .onFalse(new InstantCommand(m_elevatorSubsystem::stop));

        InputMappings.event("coDriver", "armUp")
            .onTrue(m_armSubsystem.goToAngle(Rotation2d.kZero, false));

        InputMappings.event("coDriver", "armDown")
            .onTrue(m_armSubsystem.goToAngle(Rotation2d.fromDegrees(32.4), false));

        m_coDriverController.leftBumper()
            .onTrue(m_armSubsystem.goToAngle(Rotation2d.fromDegrees(90), false));
        
        InputMappings.event("coDriver", "coralL1")
            .onTrue(new GoToLevel(m_armSubsystem, m_elevatorSubsystem, 0));

        InputMappings.event("coDriver", "coralL2")
            .onTrue(new GoToLevel(m_armSubsystem, m_elevatorSubsystem, 1));

        InputMappings.event("coDriver", "coralL3")
            .onTrue(new GoToLevel(m_armSubsystem, m_elevatorSubsystem, 2));
    
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

        NamedCommands.registerCommand("Intake Coral", 
            new GoToLevel(m_armSubsystem, m_elevatorSubsystem, 0)
                .andThen(new IntakeCoral(m_armSubsystem, m_coralIntakeSubsystem))
        );
        NamedCommands.registerCommand("Shoot Coral", m_coralIntakeSubsystem.shootCoral().withTimeout(0.5));

        Set<Subsystem> coralCommandRequirements = new HashSet<>();
        coralCommandRequirements.add(m_armSubsystem);
        coralCommandRequirements.add(m_elevatorSubsystem);

        NamedCommands.registerCommand("1st Coral", 
            new DeferredCommand(() -> new GoToLevel(m_armSubsystem, m_elevatorSubsystem, m_autonFirstCoralLevel.getSelected()), coralCommandRequirements)
        );

        NamedCommands.registerCommand("2nd Coral",
            new DeferredCommand(() -> new GoToLevel(m_armSubsystem, m_elevatorSubsystem, m_autonSecondCoralLevel.getSelected()), coralCommandRequirements)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return The command to run in autonomous.
     */
    public Command getAutonomousCommand() {
        Command selectedManually = m_manualAuton.getSelected();

        if (selectedManually != null) {
            return selectedManually;
        }

        return AutoBuilder.buildAuto(m_autonNumCorals.getSelected() + " " + m_autonStartPose.getSelected());
    }

    /**
     * Gets the {@link DriveSubsystem} of the container.
     * @return The <code>DriveSubsystem</code>.
     */
    public DriveSubsystem getDriveSubsystem() {
        return m_driveSubsystem;
    }
}
