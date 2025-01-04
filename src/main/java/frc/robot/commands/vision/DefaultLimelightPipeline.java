package frc.robot.commands.vision;

//Import subsystem(s) this command interacts with below

import frc.robot.subsystems.VisionSubsystem;

import frc.robot.Constants.VisionConstants;

//Import this so you can make this class a command
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultLimelightPipeline extends Command {

    //Import any instance variables that are passed into the file below here, such as the subsystem(s) your command interacts with.
    final VisionSubsystem m_visionSubsystem;

    /** 
     * Default command for vision subsystem. 
     * This command makes it so that when no other command is using the vision subsystem, the pipeline will automatically switch
     * back to the default pipeline.
     * This year we are only using one pipeline, so this command is kinda useless.
    */
    public DefaultLimelightPipeline(VisionSubsystem visionSubsystem){
        m_visionSubsystem = visionSubsystem;
        
        //If your command interacts with any subsystem(s), you should pass them into "addRequirements()"
        //This function makes it so your command will only run once these subsystem(s) are free from other commands.
        //This is really important as it will stop scenarios where two commands try to controll a motor at the same time.
        addRequirements(m_visionSubsystem);
    }

    // This function is called once when the command is schedueled.
    @Override
    public void initialize(){
        if(m_visionSubsystem.getPipeline() != VisionConstants.kDefaultPipeline){
            m_visionSubsystem.setPipeline(VisionConstants.kDefaultPipeline);
        }
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
}