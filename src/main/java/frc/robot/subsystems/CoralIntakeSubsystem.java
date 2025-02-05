package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CoralIntakeConstants;

/**
 * The subsystem that controls the robot's coral intake.
 */
public class CoralIntakeSubsystem {
    /** The motor controller for the coral intake motor. */
    private final SparkMax m_coralIntakeMotor = new SparkMax(CoralIntakeConstants.kCoralIntakeMotorCanId, MotorType.kBrushless);

    public CoralIntakeSubsystem() {
        SparkMaxConfig coralIntakeMotorConf = new SparkMaxConfig();
        
        // sets the configation of the motors
        coralIntakeMotorConf
            .inverted(CoralIntakeConstants.kCoralIntakeInverted)
            .idleMode(CoralIntakeConstants.kCoralIntakeIdleMode)
            .smartCurrentLimit(CoralIntakeConstants.kSmartCurrentLimit);

        m_coralIntakeMotor.configure(coralIntakeMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
