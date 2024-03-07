package frc.robot.commands.Autos;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherFeeder;

public class FeedNoteAuto extends Command {
    private LauncherFeeder _launcherFeeder;
    private Timer _timer;
    private Supplier<Boolean> _hasNoteSupplier;
    private Supplier<Boolean> _atReferenceSpeedSupplier;

     public FeedNoteAuto(LauncherFeeder launcherFeeder, Supplier<Boolean> hasNoteSupplier) {
        _launcherFeeder = launcherFeeder;
        _hasNoteSupplier = hasNoteSupplier;
        addRequirements(_launcherFeeder);
    }
    
     public FeedNoteAuto(LauncherFeeder launcherFeeder, Supplier<Boolean> hasNoteSupplier, Supplier<Boolean> atReferenceSpeedSupplier) {
        _launcherFeeder = launcherFeeder;
        _hasNoteSupplier = hasNoteSupplier;
        _atReferenceSpeedSupplier = atReferenceSpeedSupplier;
        addRequirements(_launcherFeeder);
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
        if(_atReferenceSpeedSupplier != null && _atReferenceSpeedSupplier.get()) {
                _launcherFeeder.setFeederSpeed(1);
        } else {
            _launcherFeeder.setFeederSpeed(1);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _launcherFeeder.setFeederSpeed(0);
        _timer.stop();
        _timer.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (!_hasNoteSupplier.get()) || (_timer.hasElapsed(.5));
    }

}
