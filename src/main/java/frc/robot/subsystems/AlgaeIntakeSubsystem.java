package frc.robot.subsystems;

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
    private final SparkMax m_algaeIntakeMotorRoller = new SparkMax(AlgaeIntakeConstaints.kIntakeMotorId, MotorType.kBrushless);


    public AlgaeIntakeSubsystem() {
        SparkMaxConfig RollerMotorConf = new SparkMaxConfig();
        SparkMaxConfig ArmMotorConf = new SparkMaxConfig();



        /*sets the configation of the motors*/ 
        RollerMotorConf
            .inverted(AlgaeIntakeConstaints.kleftAlgaeIntakeInverted)
            .idleMode(AlgaeIntakeConstaints.kAlgaeIntakeIdleMode)
            .smartCurrentLimit(AlgaeIntakeConstaints.kSmartCurrentLimit);
        ArmMotorConf
            .inverted(AlgaeIntakeConstaints.krightAlgaeIntakeInverted)
            .idleMode(AlgaeIntakeConstaints.kAlgaeIntakeIdleMode)
            .smartCurrentLimit(AlgaeIntakeConstaints.kSmartCurrentLimit);

            m_algaeIntakeMotorRoller.configure(RollerMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_algaeIntakeMotorArm.configure(ArmMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setMotorSpeed(double armSpeed, double rollerSpeed){
        m_algaeIntakeMotorRoller.set(rollerSpeed);
        m_algaeIntakeMotorArm.set(armSpeed);
    }

    public void setRollerSpeed(double speed){
        m_algaeIntakeMotorRoller.set(speed);
    }

    public void setArmSpeed(double speed){
        m_algaeIntakeMotorArm.set(speed);
    }

}