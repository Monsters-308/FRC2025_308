package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.ThroughBoreEncoder;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax m_armMotor = new SparkMax(0, null);
    private final ThroughBoreEncoder m_armEncoder = new ThroughBoreEncoder(0, false, 0, 0);


    SparkMaxConfig armMotorConf = new SparkMaxConfig();

    public ArmSubsystem(){
        armMotorConf
            .inverted(false)
            .smartCurrentLimit(0)
            .idleMode(null);

        
        
        m_armMotor.configure(armMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}