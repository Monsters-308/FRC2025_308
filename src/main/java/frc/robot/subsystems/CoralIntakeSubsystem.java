package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;


/**
 * The subsystem that controls the coral intake.
 */
public class CoralIntakeSubsystem extends SubsystemBase{
    /** The motor controller for the coral intake motor. */
    private final SparkMax m_coralIntakeMotor = new SparkMax(CoralIntakeConstants.kCoralIntakeMotorCanId, MotorType.kBrushless);
    private final DigitalInput m_sensor = new DigitalInput(CoralIntakeConstants.kSensorChannel);

    /**
     * Constructs a {@link CoralIntakeSubsystem} that controls the coral intake.
     */
    public CoralIntakeSubsystem() {
        SparkMaxConfig coralIntakeMotorConf = new SparkMaxConfig();
        
        // sets the configation of the motors
        coralIntakeMotorConf
            .inverted(CoralIntakeConstants.kCoralIntakeInverted)
            .idleMode(CoralIntakeConstants.kCoralIntakeIdleMode)
            .smartCurrentLimit(CoralIntakeConstants.kSmartCurrentLimit);

        m_coralIntakeMotor.configure(coralIntakeMotorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    /*
     * sets the speed of the flywheels depending on the what you pass into the pramader.
     */
    public void setCoralSpeed(double speed) {
        m_coralIntakeMotor.set(speed);
    }

    /*
     * This returns a true or false if the coral is detected.
     */
    public boolean isCoralDetected() {
        return m_sensor.get();
    }
    
    /*
     * This sets the speed of the Coral flywheels to shoot.
     */
    public Command shootCoral() {
        return runOnce(() -> setCoralSpeed(CoralIntakeConstants.kCoralIntakeSpeed));
    }
    /*
     * This sets the speed of the Coral flywheels to intake.
     */
    public Command ReverseCoral() {
        return runOnce(() -> setCoralSpeed(-CoralIntakeConstants.kCoralIntakeSpeed));
    }

    /*
     * only runs when coral is detected.
     */
    public Command intakeCoral() {
        return runOnce(() -> setCoralSpeed(0.1))
            .onlyWhile(() -> !isCoralDetected())
            .finallyDo(() -> setCoralSpeed(0));
    }

}
