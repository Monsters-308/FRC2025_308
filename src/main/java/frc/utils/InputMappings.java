package frc.utils;

import java.io.FileReader;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Loads an input mapping of the controller throgh a JSON file.
 */
public class InputMappings {
    private static JSONParser parser = new JSONParser();
    
    private static Map<String, CommandXboxController> controllers = new HashMap<>();

    private InputMappings() throws Exception {
        throw new Exception("InputMappings should never be instantiated.");
    }

    public static void registerController(String id, CommandXboxController controller) {
        controllers.put(id, controller);
    }

    public static Trigger getTrigger(String controllerId, String mapId, String triggerId) {
        try {
            JSONObject obj = (JSONObject)parser.parse(new FileReader(Filesystem.getDeployDirectory().getPath() + "/mappings/" + controllerId + "/" + mapId + ".json"));
            JSONObject triggers = (JSONObject)obj.get("triggers");
            JSONObject triggerData = (JSONObject)triggers.get(triggerId);
            String triggerType = (String)triggerData.get("button");
            double threshold = (double)triggerData.get("threshold");
            CommandXboxController controller = controllers.get(controllerId);

            Method triggerMethod = controller.getClass().getDeclaredMethod(triggerType);

            if (triggerType.endsWith("trigger")) {
                return (Trigger)triggerMethod.invoke(controller, threshold); 
            }

            return (Trigger)triggerMethod.invoke(controller);
        } catch(Exception e) {
            return null;
        }
    }
}
