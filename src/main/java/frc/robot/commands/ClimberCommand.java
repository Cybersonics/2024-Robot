package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command {
    private Climber _climber;

    private Supplier<Boolean> _xboxPovDownSupplier;
    private Supplier<Boolean> _xboxPovUpSupplier;

    public ClimberCommand(Climber climber, CommandXboxController xboxController) {
        _climber = climber;
        _xboxPovDownSupplier = () -> xboxController.povDown().getAsBoolean();
        _xboxPovUpSupplier = () -> xboxController.povUp().getAsBoolean();

        addRequirements(_climber);
    }

    public ClimberCommand(Climber climber, Supplier<Boolean> xboxPovUpSupplier, Supplier<Boolean> xboxPovDownSupplier) {
        _climber = climber;
        _xboxPovDownSupplier = xboxPovDownSupplier;
        _xboxPovUpSupplier = xboxPovUpSupplier;

        addRequirements(_climber);
    }
    
     public ClimberCommand(Climber _climber2, Object object, Object object2) {
        //TODO Auto-generated constructor stub
    }

    // Called when the command is initially scheduled.
     @Override
     public void initialize() { }
 
     // Called every time the scheduler runs while the command is scheduled.
     @Override
     public void execute() {
         if(_xboxPovDownSupplier.get()) {
             _climber.setClimberSpeed(0.5);
         } else if(_xboxPovUpSupplier.get()) {
             _climber.setClimberSpeed(-.5);
         } else {
            _climber.setClimberSpeed(0);
         }
     }
 
     // Called once the command ends or is interrupted.
     @Override
     public void end(boolean interrupted) { }
 
     // Returns true when the command should end.
     @Override
     public boolean isFinished() {
         return false;
     }
 
}
