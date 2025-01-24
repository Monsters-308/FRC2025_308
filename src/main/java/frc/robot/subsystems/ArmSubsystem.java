package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    private final SparkMax m_armMotor = new SparkMax(0, null);


    SparkMaxConfig armMotorConf = new SparkMaxConfig();

    public ArmSubsystem(){
        armMotorConf
        .inverted(false)
        .smartCurrentLimit(0)
        .idleMode(null);

        

        m_armMotor.configure(armMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic(){
        
    }

}
    

