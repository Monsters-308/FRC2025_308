// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DrivePIDConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotGotoAngle extends Command {

    protected final DriveSubsystem m_driveSubsystem;

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

    protected final double m_desiredAngle;

    protected final DoubleSupplier m_xSpeed;
    protected final DoubleSupplier m_ySpeed;

    /**
     * Uses PID to make the robot face a certain direction while still giving the driver control over the translation of the robot.
     * This command automatically ends when the driver tries to rotate the robot.
     * @param driveSubsystem The drive subsystem.
     * @param angle The angle to face.
     * @param xSpeed The xSpeed joystick input (gives driver control over translation).
     * @param ySpeed The ySpeed joystick input (gives driver control over translation).
     * @param driverRotation The rotation joystick input (disables command when driver tries to rotate).
     */
    public RobotGotoAngle(DriveSubsystem driveSubsystem, Rotation2d angle, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier driverRotation) {
        m_driveSubsystem = driveSubsystem;

        m_desiredAngle = angle.getRadians();

        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(DrivePIDConstants.kRotationTolerance);

        addRequirements(m_driveSubsystem);
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

        angleController.reset(Units.degreesToRadians(m_driveSubsystem.getHeading()));
        angleController.setGoal(m_desiredAngle);
    }

    /*
     * This function is called repeatedly when the schedueler's "run()" function is
     * called.
     * Once you want the function to end, you should set m_complete to true.
     */
    // When not overridden, this function is blank.
    @Override
    public void execute() {
        double rotation = angleController.calculate(Units.degreesToRadians(m_driveSubsystem.getHeading()));
        // rotation = MathUtil.clamp(rotation, -DrivePIDConstants.kRotationMaxOutput, DrivePIDConstants.kRotationMaxOutput);

        m_driveSubsystem.drive(
            m_xSpeed.getAsDouble(),
            m_ySpeed.getAsDouble(),
            rotation,
            true, true
        );
        
        if (angleController.atGoal()){
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
