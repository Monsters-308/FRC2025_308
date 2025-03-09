// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.HeadingConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotGotoAngle extends Command {

    private final DriveSubsystem m_driveSubsystem;

    private final PIDController pidController = new PIDController (  
        HeadingConstants.kHeadingP, 
        HeadingConstants.kHeadingI, 
        HeadingConstants.kHeadingD);

    private boolean m_complete = false;

    private final double m_desiredAngle;

    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;
    private final DoubleSupplier m_driverRotation;

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

        m_desiredAngle = angle.getDegrees();

        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_driverRotation = driverRotation;

        pidController.enableContinuousInput(-Math.PI, Math.PI);

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

        pidController.reset();
        pidController.setSetpoint(m_desiredAngle);
    }

    /*
     * This function is called repeatedly when the schedueler's "run()" function is
     * called.
     * Once you want the function to end, you should set m_complete to true.
     */
    // When not overridden, this function is blank.
    @Override
    public void execute() {

        double rotation = pidController.calculate(m_driveSubsystem.getHeading());

        rotation = MathUtil.clamp(rotation, -HeadingConstants.kHeadingMaxOutput, HeadingConstants.kHeadingMaxOutput);

        m_driveSubsystem.drive(
            -MathUtil.applyDeadband(m_xSpeed.getAsDouble(), OIConstants.kJoystickDeadband),
            -MathUtil.applyDeadband(m_ySpeed.getAsDouble(), OIConstants.kJoystickDeadband),
            rotation,
            true, true
        );
        
        // if(pidController.atSetpoint()){
        //     m_complete = true;
        // }

        if (MathUtil.applyDeadband(m_driverRotation.getAsDouble(), OIConstants.kJoystickDeadband) != 0){
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
