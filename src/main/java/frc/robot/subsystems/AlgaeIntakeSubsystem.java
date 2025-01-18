package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstaints;

/**
 * Subsystem that controls the Ground intake.
 */
public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final SparkMax m_algaeIntakeMotorRight = new SparkMax(IntakeConstaints.kIntakeMotorId, MotorType.kBrushless);
    private final SparkMax m_algaeIntakeMotorLeft = new SparkMax(IntakeConstaints.kIntakeMotorId, MotorType.kBrushless);


    public AlgaeIntakeSubsystem() {
        SparkMaxConfig leftMotorConf = new SparkMaxConfig();
        SparkMaxConfig rightMotorConf = new SparkMaxConfig();



        // sets the configation of the motors
        leftMotorConf
            .inverted(false)
            .idleMode(null)
            .smartCurrentLimit(1);
        rightMotorConf
            .inverted(false)
            .idleMode(null)
            .smartCurrentLimit(0);

    }

}