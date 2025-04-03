// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

// import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkBase.Warnings;

// import java.util.ArrayList;
// import java.util.Arrays;
import java.util.function.BooleanSupplier;
// import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PDHConstants;
// import frc.robot.utils.Elastic.Notification;
// import frc.robot.utils.Elastic.Notification.NotificationLevel;]

/** 
 * Utilities to help us with logging important information to networktables and 
 * detecting device faults.
 * The plan is to create a shuffleboard tab full of booleans that represent all
 * possible problems we can detect with code. That way, we can quickly check all of 
 * our devices by looking at a single tab.
 */
public class LoggingUtils {
    private LoggingUtils() {}

    private static final ShuffleboardTab m_loggingTab = Shuffleboard.getTab("Faults");

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

        BooleanSupplier hasNoFaults = () -> !sparkMax.hasActiveFault() && !sparkMax.hasActiveWarning();

        // Log faults and warnings
        m_loggingTab.addBoolean("Spark #" + sparkMax.getDeviceId(), hasNoFaults);

        // new Trigger(hasNoFaults).onFalse(new InstantCommand(() -> {
        //     Faults sparkFaults = sparkMax.getFaults();
        //     Warnings sparkWarnings = sparkMax.getWarnings();

        //     ArrayList<String> faultList = new ArrayList<>();
        //     if (sparkFaults.can) faultList.add("CAN fault");
        //     if (sparkFaults.escEeprom) faultList.add("boot during enable fault");
        //     if (sparkFaults.firmware) faultList.add("firmware fault");
        //     if (sparkFaults.gateDriver) faultList.add("gate driver fault");
        //     if (sparkFaults.motorType) faultList.add("motor type fault");
        //     if (sparkFaults.other) faultList.add("\"other\" fault");
        //     if (sparkFaults.sensor) faultList.add("sensor fault");
        //     if (sparkFaults.temperature) faultList.add("tempurature fault");
        //     if (sparkWarnings.brownout) faultList.add("brownout warning");
        //     if (sparkWarnings.escEeprom) faultList.add("brownout warning");
        //     if (sparkWarnings.extEeprom) faultList.add("brownout warning");
        //     if (sparkWarnings.hasReset) faultList.add("has reset warning");
        //     if (sparkWarnings.other) faultList.add("\"other\" warning");
        //     if (sparkWarnings.overcurrent) faultList.add("overcurrent warning");
        //     if (sparkWarnings.sensor) faultList.add("sensor warning");
        //     if (sparkWarnings.stall) faultList.add("stall warning");

        //     final String description = "A " + joinArray(faultList.toArray(new String[faultList.size()])) +
        //         (faultList.size() == 1 ? " has" : " have") + " been detected in SparkMax #" + sparkMax.getDeviceId() + ".";

        //     Notification notification = new Notification()
        //         .withLevel(NotificationLevel.ERROR)
        //         .withTitle("SparkMax Faults Detected!")
        //         .withDescription(description);
            
        //     Elastic.sendNotification(notification);
        // }));
    }

    /**
     * Creates a boolean on the Logging tab to let us know 
     * if the CANcoder is erroring.
     * @param canCoder The CANcoder.
     */
    public static void logCANcoder(CANcoder canCoder) {
        // Clear any sticky faults that might already exist
        canCoder.clearStickyFaults();

        BooleanSupplier hasNoFaults = () -> 
            canCoder.isConnected() &&
            !canCoder.getFault_BadMagnet(false).getValue() &&
            !canCoder.getFault_BootDuringEnable(false).getValue() &&
            !canCoder.getFault_Hardware(false).getValue() &&
            !canCoder.getFault_Undervoltage(false).getValue() &&
            !canCoder.getFault_UnlicensedFeatureInUse(false).getValue();

        m_loggingTab.addBoolean("CANcoder #" + canCoder.getDeviceID(), hasNoFaults);

        // new Trigger(hasNoFaults).onFalse(new InstantCommand(() -> {
        //     ArrayList<String> faultList = new ArrayList<>();
        //     if (canCoder.getFault_BadMagnet(false).getValue()) faultList.add("bad magnet");
        //     if (canCoder.getFault_BootDuringEnable(false).getValue()) faultList.add("boot during enable fault");
        //     if (canCoder.getFault_Hardware(false).getValue()) faultList.add("hardware fault");
        //     if (canCoder.getFault_Undervoltage(false).getValue()) faultList.add("undervoltage fault");
        //     if (canCoder.getFault_UnlicensedFeatureInUse(false).getValue()) faultList.add("unlicensed feature");

        //     final String title;
        //     final String description;
        //     if (!canCoder.isConnected()) {
        //         title = "CANcoder Disconnected!";
        //         description = "CANcoder #" + canCoder.getDeviceID() +" has been disconnected.";
        //     } else {
        //         title = "CANcoder Faults Detected!";
        //         description = "A " + joinArray(faultList.toArray(new String[faultList.size()])) +
        //             (faultList.size() == 1 ? " has" : " have") + " been detected in CANcoder #" + canCoder.getDeviceID() + ".";
        //     }

        //     Notification notification = new Notification()
        //         .withLevel(NotificationLevel.ERROR)
        //         .withTitle(title)
        //         .withDescription(description);
            
        //     Elastic.sendNotification(notification);
        // }));
    }

    // /** Helper function to see if any of the pdh channels have breaker faults. */
    // private static boolean hasBreakerFaults(PowerDistributionFaults faults) {
    //     for (int i = 0; i <= 23; i++) {
    //         if (faults.getBreakerFault(i))
    //             return true;
    //     }
    //     return false;
    // }

    // /** Helper function to see if any of the pdh channels have breaker faults. */
    // private static Integer[] getBreakerFaults(PowerDistributionFaults faults) {
    //     ArrayList<Integer> faultChannels = new ArrayList<>();
    //     for (int i = 0; i <= 23; i++) {
    //         if (faults.getBreakerFault(i))
    //             faultChannels.add(i);
    //     }
    //     return faultChannels.toArray(new Integer[faultChannels.size()]);
    // }

    /** 
     * Creates an instance of the PDH and uses it to log important information, such as 
     * the channel currents and sticky faults.
     */
    public static void logPDH() {
        // We cannot close this variable; it needs to stay open so the
        // lambdas below can use it.
        @SuppressWarnings("resource")
        final PowerDistribution pdh = new PowerDistribution(PDHConstants.kPDHCanID, ModuleType.kRev);
        
        // Clear any sticky faults that may already exist.
        pdh.clearStickyFaults();

        final BooleanSupplier hasNoPDHFaults = () -> {
            PowerDistributionFaults faults = pdh.getFaults();
            return !faults.Brownout && !faults.CanWarning;
        };
        
        // Log if the pdh is having any faults
        m_loggingTab.addBoolean("PDH", hasNoPDHFaults);

        // new Trigger(hasNoPDHFaults).onFalse(new InstantCommand(() -> {
        //     PowerDistributionFaults faults = pdh.getFaults();
        //     ArrayList<String> faultList = new ArrayList<>();
        //     if (faults.Brownout) faultList.add("brownout");
        //     if (faults.CanWarning) faultList.add("CAN warning");
        //     if (faults.HardwareFault) faultList.add("hardware fault");

        //     String description = "A " + joinArray(faultList.toArray(new String[faultList.size()])) +
        //         (faultList.size() == 1 ? " has" : " have") + " been detected.";

        //     Notification notification = new Notification()
        //         .withLevel(NotificationLevel.ERROR)
        //         .withTitle("PDH Faults Detected!")
        //         .withDescription(description);
            
        //     Elastic.sendNotification(notification);
        // }));

        // final BooleanSupplier hasNoBreakerFaults = () -> !hasBreakerFaults(pdh.getFaults());

        // // Log if any of the channels have breaker faults
        // loggingTab.addBoolean("PDH Breakers", hasNoBreakerFaults);

        // // Send notification to Elastic when a breaker fault is detected
        // new Trigger(hasNoBreakerFaults).onFalse(new InstantCommand(() -> {
        //     Integer[] faultChannels = getBreakerFaults(pdh.getFaults());
        //     String description = "A breaker fault has been detected on channel" +
        //         (faultChannels.length == 1 ? " ": "s ") + joinArray(faultChannels) + ".";

        //     Notification notification = new Notification()
        //         .withLevel(NotificationLevel.ERROR)
        //         .withTitle("Breaker Faults Detected!")
        //         .withDescription(description)
        //         .withNoAutoDismiss();
            
        //     Elastic.sendNotification(notification);
        // }));
    }

    /**
     * Creates a boolean on the Logging tab that indicates 
     * whether or not the NavX is connected.
     * @param navX The NavX
     */
    public static void logNavX(AHRS navX) {
        m_loggingTab.addBoolean("NavX", navX::isConnected);

        // new Trigger(navX::isConnected).debounce(2).onFalse(new InstantCommand(() -> {
        //     Notification notification = new Notification()
        //         .withLevel(NotificationLevel.ERROR)
        //         .withTitle("NavX Disconnected!")
        //         .withDescription("The NaxX as been disconnected.")
        //         .withNoAutoDismiss();
            
        //     Elastic.sendNotification(notification);
        // }));
    }

    public static void logCamera(PhotonCamera camera) {
        m_loggingTab.addBoolean(camera.getName(), camera::isConnected);
    }

    /**
     * Joins an array with commas and an "and".
     * @param array The array to join.
     * @return The joined array as a string.
     */
    // private static String joinArray(String[] array) {
    //     if (array == null || array.length == 0) {
    //         return "";
    //     }
    //     if (array.length == 1) {
    //         return array[0];
    //     }
    //     if (array.length == 2) {
    //         return String.join(" and ", array);
    //     }

    //     String allButLast = Arrays.stream(array)
    //             .limit(array.length - 1)
    //             .collect(Collectors.joining(", "));
        
    //     return allButLast.concat(" and ").concat(array[array.length - 1]);
    // }

    // /**
    //  * Joins an array with commas and an "and".
    //  * @param array The array to join.
    //  * @return The joined array as a string.
    //  */
    // private static String joinArray(Integer[] array) {
    //     String[] a = Arrays.toString(array).split("[\\[\\]]")[1].split(", "); 
    //     return joinArray(a);
    // }

    public static void logBattery() {
        // new Trigger(() -> RobotController.getBatteryVoltage() > 10).onFalse(new InstantCommand(() -> {
        //     Notification notification = new Notification()
        //         .withLevel(NotificationLevel.WARNING)
        //         .withTitle("Battery Low!")
        //         .withDescription("The output battery voltage is below 11 volts.");
            
        //     Elastic.sendNotification(notification);
        // }));
    }
}
