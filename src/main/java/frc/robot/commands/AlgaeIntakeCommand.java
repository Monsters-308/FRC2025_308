package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgaeIntakeCommand extends Command {
   // private final AlgaeIntakeSubsystem m_algaeIntakeSubsystem;


   /**
    * This command controls the intake an which way its rotating or the way its shooting.
    * @param algaeIntakeSubsystem
    */

   public AlgaeIntakeCommand(AlgaeIntakeSubsystem algaeIntakeSubsystem){
      // m_algaeIntakeSubsystem = algaeIntakeSubsystem;

      addRequirements(algaeIntakeSubsystem);
   }

}
