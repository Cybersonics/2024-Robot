package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command {
    private Climber _climber;
    private CommandXboxController _xboxController;

    public ClimberCommand(Climber climber, CommandXboxController xboxController) {
        _climber = climber;
        _xboxController = xboxController;

        addRequirements(_climber);
    }
    
     // Called when the command is initially scheduled.
     @Override
     public void initialize() {
     }
 
     // Called every time the scheduler runs while the command is scheduled.
     @Override
     public void execute() {
         if(_xboxController.povDown().getAsBoolean()) {
             _climber.setClimberSpeed(0.5);
         } else if(_xboxController.povUp().getAsBoolean()) {
             _climber.setClimberSpeed(-.5);
         } else {
            _climber.setClimberSpeed(0);
         }
     }
 
     // Called once the command ends or is interrupted.
     @Override
     public void end(boolean interrupted) {
     }
 
     // Returns true when the command should end.
     @Override
     public boolean isFinished() {
         return false;
     }
 
}
