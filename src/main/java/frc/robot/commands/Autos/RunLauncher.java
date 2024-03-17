package frc.robot.commands.Autos;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Launcher;

public class RunLauncher extends Command {
    
    private Launcher _launcher;
    private Timer _timer;
    private Supplier<Boolean> _isLauncherUpSupplier;

    public RunLauncher(Launcher launcher, Supplier<Boolean> isLauncherUpSupplier) {
        _launcher = launcher;
        _isLauncherUpSupplier = isLauncherUpSupplier;

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
        if (_isLauncherUpSupplier.get()) {
            _launcher.setReferenceSpeed(Constants.LauncherConstants.topFarLobRPM, Constants.LauncherConstants.bottomFarLobRPM, _isLauncherUpSupplier.get());
        } else {
            _launcher.setReferenceSpeed(Constants.LauncherConstants.topFarShotRPM, Constants.LauncherConstants.bottomFarShotRPM, _isLauncherUpSupplier.get());
        }        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // _launcher.setLauncherSpeed(0);
        _timer.stop();
        _timer.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
