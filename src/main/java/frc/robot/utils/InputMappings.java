// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.text.WordUtils;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.Elastic.Notification;
import frc.robot.utils.Elastic.Notification.NotificationLevel;

/**
 * Loads input mappings of the controllers through JSON files.
 */
public final class InputMappings {
    private InputMappings() {}

    /** The {@link JSONParser} used to parse input mappings. */
    private static final JSONParser m_parser = new JSONParser();
    
    /** The {@link Map} of IDs and controller objects. */
    private static final Map<String, CommandGenericHID> m_controllers = new HashMap<>();
    /** The {@link Map} of controller IDs and {@link SendableChooser} objects. */
    private static final Map<String, SendableChooser<String>> m_choosers = new HashMap<>();

    /**
     * Registers a <code>GenericHID</code> object with the specified ID.
     * It will be used to create <code>Trigger</code> objects when calling <code>InputMappings.event</code>
     * @param id The ID for the specified <code>GenericHID</code> object.
     * @param controller The <code>GenericHID</code> object.
     * @see GenericHID
     * @see Trigger
     * @see InputMappings#event
     * @see InputMappings#registerController(String, CommandGenericHID)
     */
    public static void registerController(String id, GenericHID controller) {
        m_controllers.put(id, new CommandGenericHID(controller.getPort()));
    }

    /**
     * Registers a <code>CommandGenericHID</code> object with the specified ID.
     * It will be used to create <code>Trigger</code> objects when calling <code>InputMappings.event</code>
     * @param id The ID for the specified <code>CommandGenericHID</code> object.
     * @param controller The <code>CommandGenericHID</code> object.
     * @see CommandGenericHID
     * @see Trigger
     * @see InputMappings#event
     * @see InputMappings#registerController(String, GenericHID)
     */
    public static void registerController(String id, CommandGenericHID controller) {
        m_controllers.put(id, controller);
    }

    /**
     * Creates a <code>Trigger</code> for the specified event on the specified controller in the currently selected mapping.
     * @param controllerId The ID of the controller.
     * @param eventId The ID of the event on the controller.
     * @return The <code>Trigger</code> which will trigger with the button defined in the selected mapping.
     * @see Trigger
     */
    public static Trigger event(String controllerId, String eventId) {
        final CommandGenericHID controller = m_controllers.get(controllerId);

        final File mappingsDirectory = Filesystem.getDeployDirectory().toPath().resolve("mappings").toFile();
        final File controllerDirectory = mappingsDirectory.toPath().resolve(controllerId).toFile();

        try {
            if (!mappingsDirectory.exists()) {
                throw new MappingsDirectoryNotFoundException();
            }

            if(!controllerDirectory.exists()) {
                throw new ControllerNotFoundException(controllerId);
            }
        } catch (MappingsDirectoryNotFoundException | ControllerNotFoundException e) {
            reportError(e);
            return new Trigger(() -> false);
        }

        final File[] mappings = controllerDirectory.listFiles();
        final Trigger[] triggers = new Trigger[mappings.length];

        for (int i = 0; i < mappings.length; i++) {
            final JSONObject obj;

            try {
                obj = (JSONObject)m_parser.parse(new FileReader(mappings[i]));
            } catch (IOException | ParseException e) {
                reportError(e);
                return new Trigger(() -> false);
            }

            final String eventButton;
            final Double threshold;

            try {
                final JSONObject mappingTriggers = (JSONObject)obj.get("events");
                final Object eventData = mappingTriggers.get(eventId);
                if (eventData == null) {
                    throw new EventNotFoundException(eventId, mappings[i].getAbsolutePath());
                }

                if (eventData.getClass() == String.class) {
                    eventButton = (String)eventData;
                    threshold = null;
                } else {
                    final JSONObject eventJsonObject = (JSONObject)eventData;
                    eventButton = (String)eventJsonObject.get("button");
                    threshold = (Double)eventJsonObject.get("threshold");
                }
            } catch (EventNotFoundException e) {
                reportError(e);
                return new Trigger(() -> false);
            }

            try {
                final Method triggerMethod;

                if ((eventButton.equals("leftTrigger") || eventButton.equals("rightTrigger")) && threshold != null) {
                    triggerMethod = controller.getClass().getMethod(eventButton, double.class);
                    triggers[i] = (Trigger)triggerMethod.invoke(controller, threshold);
                } else {
                    triggerMethod = controller.getClass().getMethod(eventButton);
                    triggers[i] = (Trigger)triggerMethod.invoke(controller);
                }
            } catch (NoSuchMethodException | SecurityException e) {
                reportError(e);
                return new Trigger(() -> false);
            } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
                reportError(e);
                return new Trigger(() -> false);
            }
        }

        Trigger combinedTrigger = triggers[0].and(() -> { 
            final String selected = getChooser(controllerId).getSelected();
            if (selected == null) return false;
            return selected.equals(mappings[0].getAbsolutePath());
        });

        for (int i = 1; i < triggers.length; i++) {
            final int iFinal = i; // so "i" can be used inside a lambda

            combinedTrigger = combinedTrigger.or(
                triggers[i].and(() -> { 
                    final String selected = getChooser(controllerId).getSelected();
                    if (selected == null) return false;
                    return selected.equals(mappings[iFinal].getAbsolutePath());
                })
            );
        }

        return combinedTrigger;
    }

    /**
     * Gets the <code>SendableChooser</code> with options for each button mapping for the specified controller.
     * Will create one if not already present.
     * @param controllerId The ID for the controller.
     * @return The <code>SendableChooser</code>.
     * @see SendableChooser
     * @see InputMappings#addChoosers
     */
    public static SendableChooser<String> getChooser(String controllerId) {
        if (m_choosers.containsKey(controllerId)) return m_choosers.get(controllerId);

        final File mappingsDirectory = Filesystem.getDeployDirectory().toPath().resolve("mappings").toFile();
        final File controllerDirectory = mappingsDirectory.toPath().resolve(controllerId).toFile();

        try {
            if (!mappingsDirectory.exists()) {
                throw new MappingsDirectoryNotFoundException();
            }

            if(!controllerDirectory.exists()) {
                throw new ControllerNotFoundException(controllerId);
            }
        } catch (MappingsDirectoryNotFoundException | ControllerNotFoundException e) {
            reportError(e);
            return null;
        }

        @SuppressWarnings("resource")
        final SendableChooser<String> chooser = new SendableChooser<>();
        
        final File[] mappings = controllerDirectory.listFiles();

        for (final File mapping : mappings) {
            String displayName;

            try {
                displayName = (String)((JSONObject)m_parser.parse(new FileReader(mapping))).get("displayName");
            } catch (IOException | ParseException e) {
                reportError(e);
                return null;
            }

            if (displayName == null) {
                displayName = WordUtils.capitalize(mapping.getName().replace(".json", ""));
            }

            if (mapping.getName().equals("default.json")) {
                chooser.setDefaultOption(displayName, mapping.getAbsolutePath());
                continue;
            }

            chooser.addOption(displayName, mapping.getAbsolutePath());
        }

        m_choosers.put(controllerId, chooser);

        return chooser;
    }

    /**
     * Adds every mapping <code>SendableChooser</code> for every controller.
     * to a new <code>ShuffleboardLayout</code> in the specified <code>ShuffleboardTab</code>.
     * @param tab The <code>ShuffleboardTab</code> to add the <code>ShuffleboardLayout</code> and <code>SendableChooser</code> objects to.
     * @see SendableChooser
     * @see ShuffleboardTab
     * @see ShuffleboardLayout
     * @see InputMappings#getChooser
     */
    public static void addChoosers(ShuffleboardTab tab) {
        final ShuffleboardLayout mappingLayout = tab.getLayout("Mappings", BuiltInLayouts.kList);
        final ShuffleboardLayout vibrationLayout = tab.getLayout("Vibrate Controllers", BuiltInLayouts.kList);

        for (final String key : m_controllers.keySet()) {
            SendableChooser<String> chooser = getChooser(key);
            if (chooser == null) {
                continue;
            }
            mappingLayout.add(WordUtils.capitalize(key), chooser);
            vibrationLayout.add(key, 
                new InstantCommand(() -> m_controllers.get(key).setRumble(RumbleType.kBothRumble, 1))
                    .andThen(new WaitCommand(2))
                    .finallyDo(() -> m_controllers.get(key).setRumble(RumbleType.kBothRumble, 0))
                    .ignoringDisable(true)
            );
        }
    }

    /**
     * Reports an error to the {@link DriverStation} and sends a notification to {@link Elastic}.
     * @param throwable A {@link Throwable} object that represents the error.
     */
    private static void reportError(Throwable throwable) {
        DriverStation.reportError(throwable.getClass().getSimpleName() + ": " + throwable.getLocalizedMessage(), true);

        Notification notification = new Notification()
            .withLevel(NotificationLevel.ERROR)
            .withTitle(throwable.getClass().getSimpleName())
            .withDescription(throwable.getLocalizedMessage())
            .withNoAutoDismiss();
        
        Elastic.sendNotification(notification);
    }

    /**
     * Thrown when <code>InputMappings</code> methods cannot find the
     * "mappings" directory in the deploy directory.
     * @see InputMappings#event
     * @see InputMappings#getChooser
     * @see InputMappings#addChoosers
     */
    public static class MappingsDirectoryNotFoundException extends Exception {
        /**
         * Creates a new {@link MappingsDirectoryNotFoundException} with the default message.
         */
        public MappingsDirectoryNotFoundException() {
            super("The \"mappings\" directory was not found. Make sure it is located in the deploy directoy.");
        }
    }

    /**
     * Thrown when <code>InputMappings</code> methods cannot find the
     * directory for a controller ID in the mappings directory.
     * @see InputMappings#event
     * @see InputMappings#getChooser
     * @see InputMappings#addChoosers
     */
    public static class ControllerNotFoundException extends Exception {
        /**
         * Creates a new {@link ControllerNotFoundException} with the
         * default message for the specified controller ID.
         * @param controllerId The controller ID that was not found.
         */
        public ControllerNotFoundException(String controllerId) {
            super("The \"" + controllerId + "\" directory was not found. Make sure it is located in the mappings directoy.");
        }
    }

    /**
     * Thrown when {@link InputMappings#event} methods cannot find an
     * event with the specified ID in a mapping
     */
    public static class EventNotFoundException extends Exception {
        /**
         * Creates a new {@link MappingsDirectoryNotFoundException} with the default message.
         * @param eventId The event ID which was not found.
         * @param mappingPath The path of the mapping JSON file in which the event was not found.
         */
        public EventNotFoundException(String eventId, String mappingPath) {
            super("No key \"" + eventId + "\" was found in \"" + mappingPath + "\". Make sure it is located in the \"events\" object in the root of the file.");
        }
    }
}
