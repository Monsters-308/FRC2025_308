package frc.utils;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Loads input mappings of the controllers through JSON files.
 */
public final class InputMappings {
    /** The {@link JSONParser} used to parse input mappings. */
    private static JSONParser m_parser = new JSONParser();
    
    /** The {@link Map} of IDs and controller objects. */
    private static Map<String, CommandXboxController> m_controllers = new HashMap<>();
    /** The {@link Map} of controller IDs and {@link SendableChooser} objects. */
    private static Map<String, SendableChooser<String>> m_choosers = new HashMap<>();

    /**
     * Cannot instantiate the type {@link InputMappings}.
     * @throws InstantiationException
     */
    private InputMappings() throws InstantiationException {
        throw new InstantiationException("Cannot instantiate the type InputMappings");
    }

    /**
     * Registers a {@link CommandXboxController} object with the specified ID.
     * @param id The ID to use with the controller.
     * @param controller The {@link CommandXboxController} object.
     */
    public static void registerController(String id, CommandXboxController controller) {
        m_controllers.put(id, controller);
    }

    /**
     * Creates a {@link Trigger} for the specified event on the specified controller in the currently selected mapping.
     * @param controllerId The ID for the controller.
     * @param eventId The ID for the event on the controller.
     * @return The {@link Trigger} which will trigger with the button defined in the selected mapping.
     */
    public static Trigger event(String controllerId, String eventId) {
        CommandXboxController controller = m_controllers.get(controllerId);

        File mappingsDirectory = Filesystem.getDeployDirectory().toPath().resolve(
            "mappings" + File.separatorChar + controllerId
        ).toFile();

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

            if (triggerType.endsWith("Trigger") && threshold != null) {
                Method triggerMethod;

                try {
                    triggerMethod = controller.getClass().getDeclaredMethod(triggerType, double.class);
                } catch (NoSuchMethodException | SecurityException e) {
                    DriverStation.reportError(e.getLocalizedMessage(), true);
                    return new Trigger(() -> false);
                }

                try {
                    triggers[i] = (Trigger)triggerMethod.invoke(controller, threshold);
                } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
                    DriverStation.reportError(e.getLocalizedMessage(), true);
                    return new Trigger(() -> false);
                }
                continue;
            }

            Method triggerMethod;

            try {
                triggerMethod = controller.getClass().getDeclaredMethod(triggerType);
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

        Trigger combinedTrigger = triggers[0].and(
            () -> getChooser(controllerId).getSelected().equals(mappings[0].getAbsolutePath())
        );

        for (int i = 1; i < triggers.length; i++) {
            final int index = i; // so "i" can be used inside a lambda

            combinedTrigger = combinedTrigger.or(
                triggers[i].and(
                    () -> getChooser(controllerId).getSelected().equals(mappings[index].getAbsolutePath())
                )
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

            File mappingsDirectory = Filesystem.getDeployDirectory().toPath().resolve(
                "mappings" + File.separatorChar + controllerId
            ).toFile();

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
                    displayName = mapping.getName().replaceAll("\\.json", "");
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
}
