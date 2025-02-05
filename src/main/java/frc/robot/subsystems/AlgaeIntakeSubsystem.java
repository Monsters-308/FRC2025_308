package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstaints;

/**
 * Subsystem that controls the algae intake.
 */
public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final SparkMax m_algaeIntakeMotorArm = new SparkMax(AlgaeIntakeConstaints.kIntakeMotorCanId, MotorType.kBrushless);
    private final SparkMax m_algaeIntakeMotorRoller = new SparkMax(AlgaeIntakeConstaints.kIntakeMotorCanId, MotorType.kBrushless);


    public AlgaeIntakeSubsystem() {
        SparkMaxConfig rollerMotorConf = new SparkMaxConfig();
        SparkMaxConfig armMotorConf = new SparkMaxConfig();
        
        // sets the configation of the motors
        rollerMotorConf
            .inverted(AlgaeIntakeConstaints.kleftAlgaeIntakeInverted)
            .idleMode(AlgaeIntakeConstaints.kAlgaeIntakeIdleMode)
            .smartCurrentLimit(AlgaeIntakeConstaints.kSmartCurrentLimit);
        armMotorConf
            .inverted(AlgaeIntakeConstaints.krightAlgaeIntakeInverted)
            .idleMode(AlgaeIntakeConstaints.kAlgaeIntakeIdleMode)
            .smartCurrentLimit(AlgaeIntakeConstaints.kSmartCurrentLimit);

        m_algaeIntakeMotorRoller.configure(rollerMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_algaeIntakeMotorArm.configure(armMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // this will set the speeds.
    public void setIntakeSpeeds(double armSpeed, double rollerSpeed) {
        m_algaeIntakeMotorRoller.set(rollerSpeed);
        m_algaeIntakeMotorArm.set(armSpeed);
    }

    public void setRollerSpeed(double speed) {
        m_algaeIntakeMotorRoller.set(speed);
    }
    
    public void setArmSpeed(double speed) {
        m_algaeIntakeMotorArm.set(speed);
    }


    /**
     * this will set the speeds in the command.
     */
    public Command startRoller() {
        return runOnce(() -> setRollerSpeed(AlgaeIntakeConstaints.kRollerSpeed));
    }

    public Command shootAlgae() {
        return runOnce(() -> setRollerSpeed(-AlgaeIntakeConstaints.kRollerSpeed));
    }

    public Command startArmSpeed() {
        //TODO: do this!!!!!! >:3
        return null;
    }

}