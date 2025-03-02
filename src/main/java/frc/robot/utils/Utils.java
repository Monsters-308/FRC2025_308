// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.Consumer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;

/**
 * General utilities used throughout the code.
 */
public class Utils {
    private Utils() {}

    /**
     * Limit the linear and angular accelerations of the chassis between setpoints.
     * @param target - the target chassis velocity for the next period
     * @param previous - the previous target chassis velocity
     * @param dt - the time interval between periods
     * @param linear_acc_lim - the maximum linear acceleration allowed
     * @param rotational_acc_lim - the maximum rotational acceleration allowed
     */
    public static void rateLimitVelocity(ChassisSpeeds target, ChassisSpeeds previous, double dt, double linear_acc_lim, double rotational_acc_lim) {
        final double
            maxVelLim = linear_acc_lim * dt,        // the maximum change in velocity allowed for the given time interval
            maxRotLim = rotational_acc_lim * dt,
            deltaVX = target.vxMetersPerSecond - previous.vxMetersPerSecond,    // the current change in velocity (components)
            deltaVY = target.vyMetersPerSecond - previous.vyMetersPerSecond,
            deltaRot = target.omegaRadiansPerSecond - previous.omegaRadiansPerSecond,
            deltaVel = Math.hypot(deltaVX, deltaVY),                  // the current change in velocity (vector magnitude)
            newDeltaVel = MathUtil.clamp(deltaVel, -maxVelLim, maxVelLim),    // the clamped magnitude
            newDeltaRot = MathUtil.clamp(deltaRot, -maxRotLim, maxRotLim),
            scale = deltaVel == 0.0 ? 1.0 : newDeltaVel / deltaVel,         // protect against div by 0 when delta velocity was (0, 0)
            newDeltaVX = deltaVX * scale,                         // rescale component deltas based on clamped magnitude
            newDeltaVY = deltaVY * scale;
        target.vxMetersPerSecond = previous.vxMetersPerSecond + newDeltaVX;       // reapply clamped changes in velocity
        target.vyMetersPerSecond = previous.vyMetersPerSecond + newDeltaVY;
        target.omegaRadiansPerSecond = previous.omegaRadiansPerSecond + newDeltaRot;
    }

    /**
     * Converts an angle from 0-360 to -180-180
     * @param degrees An angle that goes from 0 to 360 degrees
     * @return The angle from -180 to 180 degrees
     */
    public static double angleConstrain(double degrees){
        return MathUtil.inputModulus(degrees, -180, 180);
    }

    /**
     * Restricts a pose's translation to the confines of the field.
     * This isn't very useful for our practice field where we have no barriers,
     * but it will increase the accuracy of our odometry on the real field. 
     * @param pose The pose to be restricted.
     * @return The same pose but with restrictions applied.
     */
    public static Pose2d restrictPose(Pose2d pose){
        return new Pose2d(
            MathUtil.clamp(pose.getX(), 0, FieldConstants.kFieldWidthMeters),
            MathUtil.clamp(pose.getY(), 0, FieldConstants.kFieldHeightMeters),
            pose.getRotation()
        );
    }

    /**
     * Calculates the angle that point A has to be at in order to face
     * point B. This will be used for auto-aiming.
     * @param pointA The point that needs to face another point.
     * @param pointB The point being faced at by pointA.
     * @return The angle from point A to point B.
     */
    public static Rotation2d anglePoseToPose(Translation2d pointA, Translation2d pointB){
        double deltaX = pointB.getX() - pointA.getX();
        double deltaY = pointB.getY() - pointA.getY();

        // Use Math.atan2 to calculate the angle
        double angle = Math.atan2(deltaY, deltaX);

        return new Rotation2d(angle);
    }

    /**
     * Calculates the distance that point A is from point B
     * This will be used for auto-aiming.
     * @param pointA The starting point
     * @param pointB The ending point
     * @return The distance from point A to point B.
     */
    public static double getDistancePosToPos(Translation2d pointA, Translation2d pointB){
        double x1 = pointA.getX();
        double y1 = pointA.getY();

        double x2 = pointB.getX();
        double y2 = pointB.getY();

        double distance = Math.sqrt(Math.abs(Math.pow(x1-x2, 2)) + Math.abs(Math.pow(y1-y2, 2)));

        return distance;
    }

    /**
     * Configures SysID for a <code>Subsystem</code> on a <code>ShuffleboardLayout</code>.
     * @param layout The layout to add the <code>Command</code> buttons to.
     * @param initialize Runs before any SysID <code>Command</code> runs.
     * @param driveAtVoltage Drive the motors being tested at the given voltage.
     * @param subsystem The <code>Subsystem</code> that controls the motors.
     * @see Subsystem
     * @see ShuffleboardLayout
     * @see Command
     */
    public static void configureSysID(ShuffleboardLayout layout, Subsystem subsystem, Runnable initialize, Consumer<Voltage> driveAtVoltage) {
        SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                driveAtVoltage,
                null, // No log consumer, since data is recorded by URCL
                subsystem
            )
        );

        // The methods below return Command objects
        Command quasistaticForward = sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
        Command quasistaticBackward = sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
        Command dynamicForward = sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
        Command dynamicBackward = sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);

        Command all = new InstantCommand(initialize)
            .andThen(quasistaticForward)
            .andThen(quasistaticBackward)
            .andThen(dynamicForward)
            .andThen(dynamicBackward)
            .withName("Run All");

        layout.add("Quasistatic Forward", new InstantCommand(initialize).andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)));
        layout.add("Quasistatic Backward", new InstantCommand(initialize).andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)));
        layout.add("Dynamic Forward", new InstantCommand(initialize).andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)));
        layout.add("Dynamic Backward", new InstantCommand(initialize).andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)));
        layout.add("Run All", all);
    }

    /**
     * Configures SysID for a <code>Subsystem</code> on a <code>ShuffleboardLayout</code>.
     * @param layout The layout to add the <code>Command</code> buttons to.
     * @param driveAtVoltage Drive the motors being tested at the given voltage.
     * @param subsystem The <code>Subsystem</code> that controls the motors.
     * @see Subsystem
     * @see ShuffleboardLayout
     * @see Command
     */
    public static void configureSysID(ShuffleboardLayout layout, Subsystem subsystem, Consumer<Voltage> driveAtVoltage) {
        configureSysID(layout, subsystem, () -> {}, driveAtVoltage);
    }

    public static double roundToNearest(double value, int place) {
        double power = Math.pow(10, place);
        return Math.round(power * value) / power;
    }
}