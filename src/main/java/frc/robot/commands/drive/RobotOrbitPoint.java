// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DrivePIDConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Utils;

public class RobotOrbitPoint extends RobotFacePoint {
    /**
     * This command rotates the robot in space using the pose estimation compared to a given point on the field.
     * The driver still has full control over the X and Y of the robot.
     */
    public RobotOrbitPoint(DriveSubsystem driveSubsystem, DoubleSupplier approachSpeed, DoubleSupplier orbitSpeed, Translation2d point) {
        super(driveSubsystem, approachSpeed, orbitSpeed, point);
    }

    /*This function is called repeatedly when the schedueler's "run()" function is called.
     * Once you want the function to end, you should set m_complete to true.
     */
    @Override
    public void execute() {
        Translation2d pos1 = m_driveSubsystem.getPose().getTranslation(); // Position of robot on field
        Translation2d pos2 = m_point; // 2D point on field
        Rotation2d angleToTarget = Utils.anglePoseToPose(pos1, pos2); // Angle to make robot face point

        // Set pid controller to angle to make robot face point
        angleController.setSetpoint(angleToTarget.getRadians());
        
        double robotHeading = Units.degreesToRadians(m_driveSubsystem.getHeading());
        double rotation = angleController.calculate(robotHeading); //speed needed to rotate robot to set point

        rotation = MathUtil.clamp(rotation, -DrivePIDConstants.kRotationMaxOutput, DrivePIDConstants.kRotationMaxOutput); // clamp value (speed limiter)
        
        m_driveSubsystem.drive(
            m_xSpeed.getAsDouble(),
            m_ySpeed.getAsDouble(),
            rotation,
            false, true
        );   
    }
}