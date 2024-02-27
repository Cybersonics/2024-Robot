package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class LauncherSpinUp extends Command {

    private Launcher _launcher;
    private CommandXboxController _xboxController;

    public LauncherSpinUp(Launcher launcher, CommandXboxController xboxController) {
        _launcher = launcher;
        _xboxController = xboxController;
        addRequirements(_launcher);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (_xboxController.rightTrigger().getAsBoolean()) {
            _launcher.setLauncherSpeed(.83);
        } else {
            _launcher.setLauncherSpeed(0);
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
