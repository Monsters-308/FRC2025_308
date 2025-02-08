package frc.utils;

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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Loads input mappings of the controllers through JSON files.
 */
public abstract class InputMappings {
    /** The {@link JSONParser} used to parse input mappings. */
    private static JSONParser m_parser = new JSONParser();
    
    /** The {@link Map} of IDs and controller objects. */
    private static Map<String, CommandGenericHID> m_controllers = new HashMap<>();
    /** The {@link Map} of controller IDs and {@link SendableChooser} objects. */
    private static Map<String, SendableChooser<String>> m_choosers = new HashMap<>();

    private InputMappings() {}

    /**
     * Registers a {@link GenericHID} object with the specified ID.
     * @param id The ID to use with the controller.
     * @param controller The {@link GenericHID} object.
     */
    public static void registerController(String id, GenericHID controller) {
        m_controllers.put(id, new CommandGenericHID(controller.getPort()));
    }

    /**
     * Registers a {@link CommandGenericHID} object with the specified ID.
     * @param id The ID to use with the controller.
     * @param controller The {@link CommandGenericHID} object.
     */
    public static void registerController(String id, CommandGenericHID controller) {
        m_controllers.put(id, controller);
    }

    /**
     * Creates a {@link Trigger} for the specified event on the specified controller in the currently selected mapping.
     * @param controllerId The ID for the controller.
     * @param eventId The ID for the event on the controller.
     * @return The {@link Trigger} which will trigger with the button defined in the selected mapping.
     */
    public static Trigger event(String controllerId, String eventId) {
        CommandGenericHID controller = m_controllers.get(controllerId);

        File mappingsDirectory = Filesystem.getDeployDirectory().toPath().resolve("mappings").toFile();
        File controllerDirectoy = mappingsDirectory.toPath().resolve(controllerId).toFile();

        try {
            if (!mappingsDirectory.exists()) {
                throw new MappingsDirectoryNotFoundException();
            }

            if(!controllerDirectoy.exists()) {
                throw new ControllerNotFoundException(controllerId);
            }
        } catch (MappingsDirectoryNotFoundException | ControllerNotFoundException e) {
            DriverStation.reportError(e.getLocalizedMessage(), true);
            return new Trigger(() -> false);
        }

        File[] mappings = mappingsDirectory.listFiles();
        Trigger[] triggers = new Trigger[mappings.length];

        for (int i = 0; i < mappings.length; i++) {
            JSONObject obj;

            try {
                obj = (JSONObject)m_parser.parse(new FileReader(mappings[i]));
            } catch (IOException | ParseException e) {
                DriverStation.reportError(e.getLocalizedMessage(), true);
                return new Trigger(() -> false);
            }

            JSONObject mappingTriggers = (JSONObject)obj.get("events");
            JSONObject triggerData = (JSONObject)mappingTriggers.get(eventId);
            String triggerType = (String)triggerData.get("button");

            Double threshold = (Double)triggerData.get("threshold");

            Method triggerMethod;

            try {
                if (triggerType.equals("leftTrigger") || triggerType.equals("rightTrigger") && threshold != null) {
                    triggerMethod = controller.getClass().getDeclaredMethod(triggerType, double.class);
                } else {
                    triggerMethod = controller.getClass().getDeclaredMethod(triggerType);
                }
            } catch (NoSuchMethodException | SecurityException e) {
                DriverStation.reportError(e.getLocalizedMessage(), true);
                return new Trigger(() -> false);
            }
            
            try {
                triggers[i] = (Trigger)triggerMethod.invoke(controller);
            } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
                DriverStation.reportError(e.getLocalizedMessage(), true);
                return new Trigger(() -> false);
            }
        }

        Trigger combinedTrigger = triggers[0].and(() -> { 
            String selected = getChooser(controllerId).getSelected();
            if (selected == null) return false;
            return selected.equals(mappings[0].getAbsolutePath());
        });

        for (int i = 1; i < triggers.length; i++) {
            final int iFinal = i; // so "i" can be used inside a lambda

            combinedTrigger = combinedTrigger.or(
                triggers[i].and(() -> { 
                    String selected = getChooser(controllerId).getSelected();
                    if (selected == null) return false;
                    return selected.equals(mappings[iFinal].getAbsolutePath());
                })
            );
        }

        return combinedTrigger;
    }

    /**
     * Creates a {@link SendableChooser} with options for each button mapping for the specified controller.
     * @param controllerId The ID for the controller.
     * @return The {@link SendableChooser}.
     */
    public static SendableChooser<String> getChooser(String controllerId) {
        SendableChooser<String> chooser = m_choosers.get(controllerId);

        if (chooser == null) {
            chooser = new SendableChooser<>();

            File mappingsDirectory = Filesystem.getDeployDirectory().toPath().resolve("mappings").toFile();
            File controllerDirectoy = mappingsDirectory.toPath().resolve(controllerId).toFile();

            try {
                if (!mappingsDirectory.exists()) {
                    throw new MappingsDirectoryNotFoundException();
                }
    
                if(!controllerDirectoy.exists()) {
                    throw new ControllerNotFoundException(controllerId);
                }
            } catch (MappingsDirectoryNotFoundException | ControllerNotFoundException e) {
                DriverStation.reportError(e.getLocalizedMessage(), true);
                return chooser;
            }

            File[] mappings = mappingsDirectory.listFiles();

            for (File mapping : mappings) {
                String displayName;

                try {
                    displayName = (String)((JSONObject)m_parser.parse(new FileReader(mapping))).get("displayName");
                } catch (IOException | ParseException e) {
                    DriverStation.reportError(e.getLocalizedMessage(), true);
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
        }

        return chooser;
    }

    /**
     * Thrown when {@link InputMappings} cannot find the
     * "mappings" directory in the deploy directory.
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
     * Thrown when {@link InputMappings} cannot find the
     * directory for a controller ID in the mappings directory.
     */
    public static class ControllerNotFoundException extends Exception {
        /**
         * Creates a new {@link ControllerNotFoundException} with the
         * default message for the specified controller ID.
         * @param controllerId controller ID.
         */
        public ControllerNotFoundException(String controllerId) {
            super("The \"" + controllerId + "\" directory was not found. Make sure it is located in the mappings directoy.");
        }
    }
}
