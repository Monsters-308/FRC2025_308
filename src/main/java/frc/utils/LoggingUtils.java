package frc.utils;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.PDHConstants;

/** 
 * Utilities to help us with logging important information to networktables and 
 * detecting device faults.
 * The plan is to create a shuffleboard tab full of booleans that represent all
 * possible problems we can detect with code. That way, we can quickly check all of 
 * our devices by looking at a single tab.
 */
public class LoggingUtils {
    private LoggingUtils() {}

    public static final ShuffleboardTab loggingTab = Shuffleboard.getTab("Logging");

    /**
     * Turns off unused telemetry on the spark max to reduce canbus usage.
     * This function may be necessary if our logging uses a lot of the canbus.
     * @param sparkMax The spark max.
     * @param disableAbsoluteEncoder Whether or not to disable absolute encoder telemetry.
     */
    public static void reduceSparkCANUsage(SparkMax sparkMax, boolean disableAbsoluteEncoder) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.signals
            .analogPositionPeriodMs(32000)
            .analogVelocityPeriodMs(32000)
            .analogVoltagePeriodMs(32000);

        if (disableAbsoluteEncoder) {
            config.signals
                .absoluteEncoderPositionPeriodMs(32000)
                .absoluteEncoderVelocityPeriodMs(32000);
        }
           
        sparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    /**
     * Creates a boolean on the Logging tab to let us know 
     * if the spark max is erroring.
     * @param sparkMax The spark max.
     */
    public static void logSparkMax(SparkMax sparkMax) {
        // Clear any sticky faults that may already exist.
        sparkMax.clearFaults();

        // Log faults and warnings
        loggingTab.addBoolean("Spark" + sparkMax.getDeviceId(), () -> !sparkMax.hasActiveFault() && !sparkMax.hasActiveWarning());
    }

    /**
     * Creates a boolean on the Logging tab to let us know 
     * if the CANcoder is erroring.
     * @param canCoder The CANcoder.
     */
    public static void logCANcoder(CANcoder canCoder) {
        // Clear any sticky faults that might already exist
        canCoder.clearStickyFaults();

        loggingTab.addBoolean("CANcoder" + canCoder.getDeviceID(), () -> 
            canCoder.isConnected() &&
            !canCoder.getFault_BadMagnet(false).getValue() &&
            !canCoder.getFault_BootDuringEnable(false).getValue() &&
            !canCoder.getFault_Hardware(false).getValue() &&
            !canCoder.getFault_Undervoltage(false).getValue() &&
            !canCoder.getFault_UnlicensedFeatureInUse(false).getValue()
        );
    }

    /** Helper function to see if any of the pdh channels have breaker faults. */
    private static boolean hasBreakerFaults(PowerDistributionFaults faults) {
        for (int i = 0; i <= 23; i++) {
            if (faults.getBreakerFault(i))
                return true;
        }
        return false;
    }

    /** 
     * Creates an instance of the PDH and uses it to log important information, such as 
     * the channel currents and sticky faults.
     */
    public static void logPDH() {
        // We cannot close this variable; it needs to stay open so the
        // lambdas below can use it.
        @SuppressWarnings("resource")
        PowerDistribution pdh = new PowerDistribution(PDHConstants.kPDHCanID, ModuleType.kRev);
        
        // Clear any sticky faults that may already exist.
        pdh.clearStickyFaults();
        
        // Log if the pdh is having any faults
        loggingTab.addBoolean("PDH faults",
            () -> {
                PowerDistributionFaults faults = pdh.getFaults();
                return !faults.Brownout && !faults.CanWarning && !faults.HardwareFault;
            }
        );

        // Log if any of the channels have breaker faults
        loggingTab.addBoolean("Breaker faults", () -> !hasBreakerFaults(pdh.getFaults()));
    }
}
