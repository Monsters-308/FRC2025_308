// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivePIDConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Utils;

public class RobotFacePoint extends Command {

    //Import any instance variables that are passed into the file below here, such as the subsystem(s) your command interacts with.
    final DriveSubsystem m_driveSubsystem;
     
    private final PIDController angleController = new PIDController(
        DrivePIDConstants.kRotationP, 
        DrivePIDConstants.kRotationI, 
        DrivePIDConstants.kRotationD
    );
    
    //If you want to control whether or not the command has ended, you should store it in some sort of variable:
    private boolean m_complete = false;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;

    private final Translation2d m_point;

    /**
     * This command rotates the robot in space using the pose estimation compared to a given point on the field.
     * The driver still has full control over the X and Y of the robot.
     */
    public RobotFacePoint(DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, Translation2d point){
        m_driveSubsystem = driveSubsystem;
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_point = point;

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(DrivePIDConstants.kRotationTolerance);

        //If your command interacts with any subsystem(s), you should pass them into "addRequirements()"
        //This function makes it so your command will only run once these subsystem(s) are free from other commands.
        //This is really important as it will stop scenarios where two commands try to controll a motor at the same time.
        addRequirements(driveSubsystem);
    }

    /*This function is called once when the command is schedueled.
     * If you are overriding "isFinished()", you should probably use this to set m_complete to false in case a command object is 
     * called a second time.
     */
    //When not overridden, this function is blank.
    @Override
    public void initialize() {
        angleController.reset();
        m_complete = false;
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
        angleController.setSetpoint(angleToTarget.getDegrees());
        
        double robotHeading = m_driveSubsystem.getHeading(); //navx

        double rotation = angleController.calculate(robotHeading); //speed needed to rotate robot to set point

        rotation = MathUtil.clamp(rotation, -DrivePIDConstants.kRotationMaxOutput, DrivePIDConstants.kRotationMaxOutput); // clamp value (speed limiter)
        
        m_driveSubsystem.drive(
            m_xSpeed.getAsDouble(),
            m_ySpeed.getAsDouble(),
            rotation,
            true, true
        );
            
    }

    @Override
    public boolean isFinished() {
        return m_complete;
    }
}