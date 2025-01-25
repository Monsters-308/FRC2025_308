package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * Absolute encoder class for a duty cycle.
 */
public class ThroughBoreEncoder extends DutyCycleEncoder {
    /** Whether the encoder is inverted. */
    private boolean m_inverted;
    /** The offset to apply to the returned angle values in degrees. */
    private double m_angleOffset;

    /**
     * A duty cycle encoder specifically for the rev through bore encoder.
     * @param channel The channel on the roborio (where the through bore encoder is plugged into on the roborio 1-10).
     * @param angleOffset The offset to apply to the returned angle values in degrees.
     * @param inverted Whether or not to negate the encoder values.
     * @param period The period of the duty cycle.
     */
    public ThroughBoreEncoder(int channel, boolean inverted, double angleOffset, int period) {
        super(channel);
        this.m_inverted = inverted;
        this.m_angleOffset = angleOffset; // Initialize the angle offset

        super.setDutyCycleRange(1 / period, 
            (period - 1) / period);
    }
    
    /** 
     * Gets the current value of the encoder in degrees.
     */
    public double getDegrees() {
        return SwerveUtils.angleConstrain((m_inverted ? -1 : 1) * 
            super.get() * 360 + m_angleOffset);
    }

    /** 
     * Gets the current value of the encoder in radians.
     */
    public double getRadians() {
        return getDegrees() * (Math.PI/180);
    }

    /** 
     * Gets the current value of the encoder in number of rotations.
     */
    public double getRotations() {
        return getDegrees() / 360;
    }

    /**
     * Gets the current value of the encoder as a Rotation2D object.
     */
    public Rotation2d getRotation2D() {
        return Rotation2d.fromDegrees(getDegrees());
    }
}