package frc.robot.subsystems;

import java.security.AlgorithmConstraints;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstaints;

/**
 * Subsystem that controls the Ground intake.
 */
public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final SparkMax m_algaeIntakeMotorArm = new SparkMax(AlgaeIntakeConstaints.kIntakeMotorId, MotorType.kBrushless);
    private final SparkMax m_algaeIntakeMotorLeft = new SparkMax(AlgaeIntakeConstaints.kIntakeMotorId, MotorType.kBrushless);


    public AlgaeIntakeSubsystem() {
        SparkMaxConfig RollerMotorConf = new SparkMaxConfig();
        SparkMaxConfig ArmMotorConf = new SparkMaxConfig();



        // sets the configation of the motors
        RollerMotorConf
            .inverted(AlgaeIntakeConstaints.kleftAlgaeIntakeInverted)
            .idleMode(null)
            .smartCurrentLimit(1);
        ArmMotorConf
            .inverted(AlgaeIntakeConstaints.krightAlgaeIntakeInverted)
            .idleMode(null)
            .smartCurrentLimit(0);

            m_algaeIntakeMotorLeft.configure(RollerMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_algaeIntakeMotorArm.configure(ArmMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic(){

    }

}