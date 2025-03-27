// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DrivePIDConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotGotoFieldPose extends Command {
    protected final DriveSubsystem m_driveSubsystem;

    protected final ProfiledPIDController controllerX = new ProfiledPIDController(  
        DrivePIDConstants.kTranslationP, 
        DrivePIDConstants.kTranslationI, 
        DrivePIDConstants.kTranslationD, 
        new Constraints(
            DrivePIDConstants.kTranslationMaxSpeed,
            DrivePIDConstants.kTranslationMaxAcceleration
        )
    );

    protected final ProfiledPIDController controllerY = new ProfiledPIDController(  
        DrivePIDConstants.kTranslationP, 
        DrivePIDConstants.kTranslationI, 
        DrivePIDConstants.kTranslationD, 
        new Constraints(
            DrivePIDConstants.kTranslationMaxSpeed,
            DrivePIDConstants.kTranslationMaxAcceleration
        )
    );

    protected final ProfiledPIDController angleController = new ProfiledPIDController(  
        DrivePIDConstants.kRotationP, 
        DrivePIDConstants.kRotationI, 
        DrivePIDConstants.kRotationD, 
        new Constraints(
            DrivePIDConstants.kRotationMaxSpeed,
            DrivePIDConstants.kRotationMaxAcceleration
        )
    );

    protected boolean m_complete = false;

    protected Pose2d m_desiredRobotPose;

    /** 
     * Uses PID to make the robot go to a certain postion relative to the field.  
     */
    public RobotGotoFieldPose(DriveSubsystem driveSubsystem, Pose2d desiredRobotPose) {
        m_driveSubsystem = driveSubsystem;

        m_desiredRobotPose = desiredRobotPose;

        controllerX.setTolerance(DrivePIDConstants.kTranslationTolerance);
        controllerY.setTolerance(DrivePIDConstants.kTranslationTolerance);
        angleController.setTolerance(DrivePIDConstants.kRotationTolerance);

        angleController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(m_driveSubsystem);
    }

    /** 
     * Uses PID to make the robot go to a certain postion relative to the field.  
     */
    public RobotGotoFieldPose(DriveSubsystem driveSubsystem, double xPosition, double yPosition, double angle) {
        this(driveSubsystem, new Pose2d(xPosition, yPosition, new Rotation2d(angle)));
    }

    // When not overridden, this function is blank.
    @Override
    public void initialize() {
        m_complete = false;

        Pose2d currentPose = m_driveSubsystem.getPose();

        controllerX.reset(currentPose.getX());
        controllerY.reset(currentPose.getY());
        angleController.reset(currentPose.getRotation().getRadians());

        controllerX.setGoal(m_desiredRobotPose.getX());
        controllerY.setGoal(m_desiredRobotPose.getY());
        angleController.setGoal(m_desiredRobotPose.getRotation().getRadians());
    }

    // When not overridden, this function is blank.
    @Override
    public void execute() {
        Pose2d currentPos = m_driveSubsystem.getPose();

        double xSpeed = controllerX.calculate(currentPos.getTranslation().getX());
        double ySpeed = controllerY.calculate(currentPos.getTranslation().getY());
        double angleSpeed = angleController.calculate(Units.degreesToRadians(m_driveSubsystem.getHeading()));


        m_driveSubsystem.drive(
            xSpeed,
            ySpeed,
            angleSpeed,
            true, false
        );
        
        m_complete = controllerX.atGoal() && controllerY.atGoal() && angleController.atGoal();
    }

    // When not overridden, this function is blank.
    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, false, false);
    }

    // When not overridden, this function returns false.
    @Override
    public boolean isFinished() {
        return m_complete;
    }
}
