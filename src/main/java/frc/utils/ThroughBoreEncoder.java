package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

/**
 * Absolute encoder class for a duty cycle.
 */
public class ThroughBoreEncoder extends DutyCycleEncoder {
    /** Whether the encoder is inverted. */
    private boolean m_inverted;
    /** The offset to apply to the returned angle values in degrees. */
    private Rotation2d m_angleOffset;

    /** The last known measurement. */
    private Rotation2d m_lastMeasurement;
    /** The time of the last measurement. */
    private double m_lastTime;

    /**
     * A duty cycle encoder specifically for the rev through bore encoder.
     * @param channel The channel on the roborio (where the through bore encoder is plugged into on the roborio 1-10).
     * @param angleOffset The offset to apply to the returned angle values.
     * @param inverted Whether or not to negate the encoder values.
     * @param period The period of the duty cycle.
     */
    public ThroughBoreEncoder(int channel, boolean inverted, Rotation2d angleOffset, int period) {
        super(channel);
        m_inverted = inverted;
        m_angleOffset = angleOffset; // Initialize the angle offset

        setDutyCycleRange(1 / period, 
            (period - 1) / period);

        getDegrees(); // Sets the last time and measurement.
    }
    
    /** 
     * Gets the current value of the encoder in degrees.
     * @return The rotation in degrees.
     */
    public double getDegrees() {
        double degrees = Utils.angleConstrain((m_inverted ? -1 : 1) * 
            super.get() * 360 + m_angleOffset.getDegrees());
        m_lastMeasurement = Rotation2d.fromDegrees(degrees);
        m_lastTime = Timer.getTimestamp();
        return degrees;
    }

    /** 
     * Gets the current value of the encoder in radians.
     * @return The rotation in radians.
     */
    public double getRadians() {
        return Units.degreesToRadians(getDegrees());
    }

    /** 
     * Gets the current value of the encoder in number of rotations.
     * @return The number of rotations.
     */
    public double getRotations() {
        return Units.degreesToRotations(getDegrees());
    }

    /**
     * Gets the current value of the encoder as a Rotation2D object.
     * @return The rotation object.
     */
    public Rotation2d getRotation2D() {
        return Rotation2d.fromDegrees(getDegrees());
    }

    /**
     * Gets the rate of change since the last measurement.
     * @return The rate of change in rotations per second.
     */
    public Rotation2d getRate() {
        return Rotation2d.fromRotations(
            (getRotations() - m_lastMeasurement.getRotations()) /
            (Timer.getTimestamp() - m_lastTime)
        );
    }
}