package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
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
    /*
     * Sets the speed of the motor.
     */
    public void setSpeed(){
        m_armMotor.set(ArmConstaints.kArmMotorSpeed);
    }

    /**
     * This function gets the angle of where the angle of the arm is currently at.
     * @return
     */
    public Rotation2d getAngle(){
        return m_armEncoder.getRotation2D();
    } 

    
    
}

