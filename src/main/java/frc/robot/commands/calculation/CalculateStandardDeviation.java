// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.calculation;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import org.apache.commons.math3.stat.descriptive.moment.StandardDeviation;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Calculates the standard deviation of a value over time.
 */
public class CalculateStandardDeviation extends Command {
    /** A double supplier that returns the values to calculate the standard deviation of. */
    private final DoubleSupplier m_valueSupplier;
    /** A double consumer that processes live updates if then standard deviation value over time. */
    private final DoubleConsumer m_update;
    /** A double consumer that processes the standard deviation once the {@link Command} finishes. */
    private final DoubleConsumer m_callback;

    /** A {@link StandardDeviation} object that calculates the standard deviations. */
    private final StandardDeviation m_stdDev = new StandardDeviation();

    /** Creates a new {@link CalculateStandardDeviation} to calculate the standard deviation of a value over time.
     * @param valueSupplier A double supplier that returns the values to calculate the standard deviation of.
     * @param update A double consumer that processes live updates if then standard deviation value over time.
     * @param callback A double consumer that processes the standard deviation once the {@link Command} finishes.
     */
    public CalculateStandardDeviation(DoubleSupplier valueSupplier, DoubleConsumer update, DoubleConsumer callback) {
        m_valueSupplier = valueSupplier;
        m_update = update;
        m_callback = callback;

        setName("Calculate");
    }

    /** Creates a new {@link CalculateStandardDeviation} to calculate the standard deviation of a value over time.
     * @param valueSupplier A double supplier that returns the values to calculate the standard deviation of.
     * @param callback A double consumer that processes the standard deviation once the {@link Command} finishes.
     */
    public CalculateStandardDeviation(DoubleSupplier valueSupplier, DoubleConsumer callback) {
        this(valueSupplier, value -> {}, callback);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_stdDev.increment(m_valueSupplier.getAsDouble());
        m_update.accept(m_stdDev.getResult());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_callback.accept(m_stdDev.getResult());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
