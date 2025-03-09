// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DrivePIDConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.Utils;

public class RobotGotoFieldPos extends Command {
    private final DriveSubsystem m_driveSubsystem;

    private final PIDController pidControllerX = new PIDController(
        DrivePIDConstants.kTranslationP, 
        DrivePIDConstants.kTranslationI, 
        DrivePIDConstants.kTranslationD
    );

    private final PIDController pidControllerY = new PIDController(
        DrivePIDConstants.kTranslationP, 
        DrivePIDConstants.kTranslationI, 
        DrivePIDConstants.kTranslationD
    );

    private final PIDController pidControllerAngle = new PIDController(
        DrivePIDConstants.kRotationP, 
        DrivePIDConstants.kRotationI, 
        DrivePIDConstants.kRotationD
    );
    private boolean m_complete = false;

    private final Pose2d m_desiredRobotPos;
    private final boolean m_allianceRelative;

    /** 
     * Uses PID to make the robot go to a certain postion relative to the field.  
     */
    public RobotGotoFieldPos(DriveSubsystem driveSubsystem, Pose2d desiredRobotoPos, boolean allianceRelative) {
        m_driveSubsystem = driveSubsystem;

        m_desiredRobotPos = desiredRobotoPos;

        m_allianceRelative = allianceRelative;

        pidControllerX.setTolerance(DrivePIDConstants.kTranslationTolerance);
        pidControllerY.setTolerance(DrivePIDConstants.kTranslationTolerance);
        pidControllerAngle.setTolerance(DrivePIDConstants.kRotationTolerance);

        pidControllerAngle.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(m_driveSubsystem);
    }

    /** 
     * Uses PID to make the robot go to a certain postion relative to the field.  
     */
    public RobotGotoFieldPos(DriveSubsystem driveSubsystem, double xPosition, double yPosition, double angle, boolean allianceRelative) {
        this(driveSubsystem, new Pose2d(xPosition, yPosition, new Rotation2d(angle)), allianceRelative);
    }

    /*
     * This function is called once when the command is schedueled.
     * If you are overriding "isFinished()", you should probably use this to set
     * m_complete to false so the command doesn't
     * instantly end.
     */
    // When not overridden, this function is blank.
    @Override
    public void initialize() {
        m_complete = false;
        pidControllerX.reset();
        pidControllerY.reset();
        pidControllerAngle.reset();

        pidControllerX.reset();
        pidControllerY.reset();
        pidControllerAngle.reset();

        Pose2d m_updatedRobotPos = !m_allianceRelative ? FieldUtils.flipRed(m_desiredRobotPos) : m_desiredRobotPos;

        pidControllerX.setSetpoint(m_updatedRobotPos.getX());
        pidControllerY.setSetpoint(m_updatedRobotPos.getY());
        pidControllerAngle.setSetpoint(Utils.angleConstrain(m_updatedRobotPos.getRotation().getDegrees()));
    }

    /*
     * This function is called repeatedly when the schedueler's "run()" function is
     * called.
     * Once you want the function to end, you should set m_complete to true.
     */
    // When not overridden, this function is blank.
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
        
        if(pidControllerX.atSetpoint() && pidControllerY.atSetpoint() && pidControllerAngle.atSetpoint()){
            m_complete = true;
        }
    }

    /*
     * This function is called once when the command ends.
     * A command ends either when you tell it to end with the "isFinished()"
     * function below, or when it is interupted.
     * Whether a command is interrupted or not is determined by
     * "boolean interrupted."
     * Things initialized in "initialize()" should be closed here.
     */
    // When not overridden, this function is blank.
    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, false, false);
    }

    /*
     * This fuction is used to tell the robot when the command has ended.
     * This function is called after each time the "execute()" function is ran.
     * Once this function returns true, "end(boolean interrupted)" is ran and the
     * command ends.
     * It is recommended that you don't use this for commands that should run
     * continuously, such as drive commands.
     */
    // When not overridden, this function returns false.
    @Override
    public boolean isFinished() {
        return m_complete;
    }
}
