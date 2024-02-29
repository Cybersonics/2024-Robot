package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LauncherFeeder;

public class AmpShot extends Command {
    private Launcher _launcher;
    private LauncherFeeder _launcherFeeder;
    private CommandXboxController _xboxController;

    public AmpShot(Launcher launcher, LauncherFeeder launcherFeeder, CommandXboxController xboxController) {
        _launcher = launcher;
        _launcherFeeder = launcherFeeder;
        _xboxController = xboxController;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(_launcher, _launcherFeeder);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(_xboxController.povLeft().getAsBoolean()) {
            _launcher.setLauncherSpeed(.3);
            _launcherFeeder.setFeederSpeed(.35);
        } else {
            _launcher.setLauncherSpeed(0);
            _launcherFeeder.setFeederSpeed(0);
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
