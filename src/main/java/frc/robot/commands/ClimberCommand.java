package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command {
    private Climber _climber;
    private CommandXboxController _xboxController;

    private Trigger _xboxPovDown;
    private Trigger _xboxPovUp;

    public ClimberCommand(Climber climber, CommandXboxController xboxController) {
        _climber = climber;
        _xboxController = xboxController;

        addRequirements(_climber);
    }
    
     // Called when the command is initially scheduled.
     @Override
     public void initialize() {
        _xboxPovUp = _xboxController.povDown();
        _xboxPovDown = _xboxController.povUp();
     }
 
     // Called every time the scheduler runs while the command is scheduled.
     @Override
     public void execute() {
         if(_xboxPovDown.getAsBoolean()) {
             _climber.setClimberSpeed(0.5);
         } else if(_xboxPovUp.getAsBoolean()) {
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
