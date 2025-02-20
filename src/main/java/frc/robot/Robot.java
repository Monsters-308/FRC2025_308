// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.urcl.URCL;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the {@link TimedRobot} documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /** The {@link Command} that follows the selected auton path. */
    private Command m_autonomousCommand;

    /** The {@link RobotContainer} for configuring elements of the robot. */
    private RobotContainer m_robotContainer;

    /**
     * Constructs a new {@link Robot}.
     */
    public Robot() {
        // Instantiate our RobotContainer. This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        // Turn off controller warnings. These are more annoying than they are helpful.
        // These will still display at competitions.
        DriverStation.silenceJoystickConnectionWarning(true);

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        URCL.start();

        WebServer.start(5800, Filesystem.getDeployDirectory().toPath().resolve("layouts").toString());
    }

    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods. This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        // Set drivetrain to Coast mode when disabled
        m_robotContainer.driveSubsystem.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        // Set drivetrain to Brake mode when enabled.
        m_robotContainer.driveSubsystem.setIdleMode(IdleMode.kBrake);

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // Set drivetrain to Brake mode when enabled.
        m_robotContainer.driveSubsystem.setIdleMode(IdleMode.kBrake);

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

    }

    @Override
    public void testPeriodic() {}
}
