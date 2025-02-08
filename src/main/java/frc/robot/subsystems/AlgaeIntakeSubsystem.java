package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

/**
 * Subsystem that controls the algae intake.
 */
public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final SparkMax m_algaeIntakeMotorArm = new SparkMax(AlgaeIntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);
    private final SparkMax m_algaeIntakeMotorRoller = new SparkMax(AlgaeIntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);

    private final RelativeEncoder m_algaeIntakeArmEncoder;

    public AlgaeIntakeSubsystem() {
        SparkMaxConfig rollerMotorConf = new SparkMaxConfig();
        SparkMaxConfig armMotorConf = new SparkMaxConfig();
        
        // sets the configation of the motors
        rollerMotorConf
            .inverted(AlgaeIntakeConstants.kAlgaeIntakeRollerInverted)
            .idleMode(AlgaeIntakeConstants.kAlgaeIntakeIdleMode)
            .smartCurrentLimit(AlgaeIntakeConstants.kSmartCurrentLimit);
        armMotorConf
            .inverted(AlgaeIntakeConstants.kAlgaeIntakeArmInverted)
            .idleMode(AlgaeIntakeConstants.kAlgaeIntakeIdleMode)
            .smartCurrentLimit(AlgaeIntakeConstants.kSmartCurrentLimit);

        m_algaeIntakeMotorRoller.configure(rollerMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_algaeIntakeMotorArm.configure(armMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_algaeIntakeArmEncoder = m_algaeIntakeMotorArm.getEncoder();
    }

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

    public Command intakeAlgae() {
        return runOnce(() -> setRollerSpeed(AlgaeIntakeConstants.kRollerSpeed));
    }

    public Command shootAlgae() {
        return runOnce(() -> setRollerSpeed(-AlgaeIntakeConstants.kRollerSpeed));
    }

    public Command armIn() {
        return runOnce(() -> setArmSpeed(-AlgaeIntakeConstants.kArmSpeed))
            .withTimeout(AlgaeIntakeConstants.kArmTimeout)
            .finallyDo(() -> setArmSpeed(0));
    }

    public Command armOut() {
        return runOnce(() -> setArmSpeed(AlgaeIntakeConstants.kArmSpeed))
            .withTimeout(AlgaeIntakeConstants.kArmTimeout)
            .finallyDo(() -> setArmSpeed(0));
    }

    public Command armToggle() {
        return runOnce(() -> setArmSpeed(
            m_algaeIntakeArmEncoder.getPosition() < AlgaeIntakeConstants.kArmErrorThreshold ? AlgaeIntakeConstants.kArmSpeed : -AlgaeIntakeConstants.kArmSpeed))
            .withTimeout(AlgaeIntakeConstants.kArmTimeout)
            .finallyDo(() -> setArmSpeed(0));
    }
}