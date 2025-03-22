// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.DrivePIDConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.Utils;

/**
 * Uses PID to make the robot go to the nearest auto align position.
 */
public class AutoAlignOld extends Command {
    private final DriveSubsystem m_driveSubsystem;

    /** The {@link PIDController} for the robot's x-coordinate. */
    private final PIDController pidControllerX = new PIDController(
        DrivePIDConstants.kTranslationP, 
        DrivePIDConstants.kTranslationI, 
        DrivePIDConstants.kTranslationD
    );

    /** The {@link PIDController} for the robot's y-coordinate. */
    private final PIDController pidControllerY = new PIDController(
        DrivePIDConstants.kTranslationP, 
        DrivePIDConstants.kTranslationI, 
        DrivePIDConstants.kTranslationD
    );

    /** The {@link PIDController} for the robot's angle. */
    private final PIDController pidControllerAngle = new PIDController(
        DrivePIDConstants.kRotationP, 
        DrivePIDConstants.kRotationI, 
        DrivePIDConstants.kRotationD
    );
    
    /** Whether the {@link Command} has finished. */
    private boolean m_complete = false;

    /** The disired position of the robot, set to the closest auto align point. */
    private Pose2d m_desiredRobotPos = null;

    /** 
     * Creates an {@link AutoAlign} object that uses PID to make the robot go to the nearest auto align position.
     */
    public AutoAlignOld(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;

        pidControllerX.setTolerance(DrivePIDConstants.kTranslationTolerance);
        pidControllerY.setTolerance(DrivePIDConstants.kTranslationTolerance);
        pidControllerAngle.setTolerance(DrivePIDConstants.kRotationTolerance);

        pidControllerAngle.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(m_driveSubsystem);
    }
    
    @Override
    public void initialize() {
        m_complete = false;

        Pose2d robotPose = m_driveSubsystem.getPose();
        Double smallestDst = null;

        boolean flipPose = robotPose.getX() > FieldConstants.kFieldWidthMeters / 2;

        for (Pose2d pose : FieldConstants.kAutoAlignPositions) {
            pose = flipPose ? FieldUtils.flip(pose) : pose;
            double dst = Utils.getDistancePosToPos(robotPose.getTranslation(), pose.getTranslation());
            if (smallestDst == null || dst < smallestDst) {
                smallestDst = dst;
                m_desiredRobotPos = pose;
            }
        }

        pidControllerX.reset();
        pidControllerY.reset();
        pidControllerAngle.reset();

        pidControllerX.setSetpoint(m_desiredRobotPos.getX());
        pidControllerY.setSetpoint(m_desiredRobotPos.getY());
        pidControllerAngle.setSetpoint(Utils.angleConstrain(m_desiredRobotPos.getRotation().getDegrees()));
    }

    @Override
    public void execute() {
        Pose2d currentPos = m_driveSubsystem.getPose();

        double xSpeed = pidControllerX.calculate(currentPos.getTranslation().getX());
        double ySpeed = pidControllerY.calculate(currentPos.getTranslation().getY());
        double angleSpeed = pidControllerAngle.calculate(m_driveSubsystem.getHeading());

        xSpeed = MathUtil.clamp(xSpeed, -DrivePIDConstants.kTranslationMaxOutput, DrivePIDConstants.kTranslationMaxOutput);
        ySpeed = MathUtil.clamp(ySpeed, -DrivePIDConstants.kTranslationMaxOutput, DrivePIDConstants.kTranslationMaxOutput);
        angleSpeed = MathUtil.clamp(angleSpeed, -DrivePIDConstants.kRotationMaxOutput, DrivePIDConstants.kRotationMaxOutput);


        m_driveSubsystem.drive(
            xSpeed,
            ySpeed,
            angleSpeed,
            true, false
        );
        
        if(pidControllerX.atSetpoint() && pidControllerY.atSetpoint() && pidControllerAngle.atSetpoint()) {
            m_complete = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return m_complete;
    }
}
