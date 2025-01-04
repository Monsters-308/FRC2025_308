package frc.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveUtils {

    /**
     * Limit the linear and angular accelerations of the chassis between setpoints.
     * @param target - the target chassis velocity for the next period
     * @param previous - the previous target chassis velocity
     * @param dt - the time interval between periods
     * @param linear_acc_lim - the maximum linear acceleration allowed
     * @param rotational_acc_lim - the maximum rotational acceleration allowed
     */
    public static void RateLimitVelocity(ChassisSpeeds target, ChassisSpeeds previous, double dt, double linear_acc_lim, double rotational_acc_lim) {
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
}