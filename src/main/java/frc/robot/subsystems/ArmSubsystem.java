package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstaints;
import frc.utils.ThroughBoreEncoder;

public class ArmSubsystem extends SubsystemBase{
    private final SparkMax m_armMotor = new SparkMax(0, null);
    private final ThroughBoreEncoder m_armEncoder = new ThroughBoreEncoder(ArmConstaints.kArmEncoderId, ArmConstaints.kEncoderInverted, ArmConstaints.kEncoderAngleOffset, ArmConstaints.kArmDutyCyclePeriod);


    SparkMaxConfig armMotorConf = new SparkMaxConfig();

    public ArmSubsystem(){
        armMotorConf
        .inverted(ArmConstaints.kArmMotorInvered)
        .smartCurrentLimit(ArmConstaints.kSmartCurrentLimit)
        .idleMode(ArmConstaints.kIdleMode);

    

        m_armMotor.configure(armMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic(){

    }

    public void setSpeed(){
        m_armMotor.set(ArmConstaints.kArmMotorSpeed);
    }

    
    
}

