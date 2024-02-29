package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class LauncherSpinUpAuto extends Command {
    
    private Launcher _launcher;
    private Timer _timer;

    public LauncherSpinUpAuto(Launcher launcher) {
        _launcher = launcher;
        addRequirements(_launcher);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _timer = new Timer();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        _timer.start();
        _launcher.setLauncherSpeed(.75);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _launcher.setLauncherSpeed(0);
        _timer.stop();
        _timer.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return _timer.hasElapsed(.25);
    }
}
