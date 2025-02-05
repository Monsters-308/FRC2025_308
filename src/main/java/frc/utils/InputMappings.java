package frc.utils;

import java.io.File;
import java.io.FileReader;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Loads an input mapping of the controller throgh a JSON file.
 */
public class InputMappings {
    private static JSONParser m_parser = new JSONParser();
    
    private static Map<String, CommandXboxController> m_controllers = new HashMap<>();
    private static Map<String, SendableChooser<String>> m_choosers = new HashMap<>();

    private InputMappings() throws Exception {
        throw new Exception("InputMappings should never be instantiated.");
    }

    public static void registerController(String id, CommandXboxController controller) {
        m_controllers.put(id, controller);
    }

    public static Trigger getTrigger(String controllerId, String triggerId) {
        try {
            CommandXboxController controller = m_controllers.get(controllerId);

            File mappingsDirectory = Filesystem.getDeployDirectory().toPath().resolve(
                "mappings" + File.separatorChar + controllerId
            ).toFile();

            File[] mappings = mappingsDirectory.listFiles();
            Trigger[] triggers = new Trigger[mappings.length];

            for (int i = 0; i < mappings.length; i++) {
                JSONObject obj = (JSONObject)m_parser.parse(new FileReader(mappings[i]));
                JSONObject mappingTriggers = (JSONObject)obj.get("triggers");
                JSONObject triggerData = (JSONObject)mappingTriggers.get(triggerId);
                String triggerType = (String)triggerData.get("button");

                Double threshold = (Double)triggerData.get("threshold");

                if (triggerType.endsWith("Trigger") && threshold != null) {
                    Method triggerMethod = controller.getClass().getDeclaredMethod(triggerType, double.class);

                    triggers[i] = (Trigger)triggerMethod.invoke(controller, threshold);
                    continue;
                }

                Method triggerMethod = controller.getClass().getDeclaredMethod(triggerType);
                triggers[i] = (Trigger)triggerMethod.invoke(controller);
            }

            Trigger combinedTrigger = triggers[0].and(
                () -> getChooser(controllerId).getSelected() == mappings[0].getAbsolutePath()
            );

            for (int i = 1; i < triggers.length; i++) {
                final int index = i; // so "i" can be used inside a lambda

                combinedTrigger = combinedTrigger.or(
                    triggers[i].and(
                        () -> getChooser(controllerId).getSelected() == mappings[index].getAbsolutePath()
                    )
                );
            }

            return combinedTrigger;
        } catch(Exception e) {
            return null;
        }
    }

    public static SendableChooser<String> getChooser(String controllerId) {
        SendableChooser<String> chooser = m_choosers.get(controllerId);

        if (chooser == null) {
            try {
                chooser = new SendableChooser<>();

                File mappingsDirectory = Filesystem.getDeployDirectory().toPath().resolve(
                    "mappings" + File.separatorChar + controllerId
                ).toFile();

                File[] mappings = mappingsDirectory.listFiles();

                for (File mapping : mappings) {
                    String displayName;

                    displayName = (String)((JSONObject)m_parser.parse(new FileReader(mapping))).get("displayName");

                    if (displayName == null) {
                        displayName = mapping.getName().replaceAll("\\.json", "");
                    }

                    if (mapping.getName() == "default.json") {
                        chooser.setDefaultOption(displayName, mapping.getAbsolutePath());
                        continue;
                    }

                    chooser.addOption(displayName, mapping.getAbsolutePath());
                }

                m_choosers.put(controllerId, chooser);
            } catch (Exception e) {
                return null;
            }
        }

        return chooser;
    }
}
