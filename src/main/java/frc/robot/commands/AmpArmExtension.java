package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pneumatics;

public class AmpArmExtension extends Command {
    private Pneumatics _pneumatics;
    private Supplier<Boolean> _hasNoteSupplier;

    private Timer _timer;

    public AmpArmExtension(Pneumatics pneumatics, Supplier<Boolean> hasNoteSupplier) {
        _pneumatics = pneumatics;
        _hasNoteSupplier = hasNoteSupplier;
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
        if(!_pneumatics.IsArmOut()) {
            _pneumatics.ampArmToggle();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _pneumatics.ampArmIn();
        _timer.stop();
        _timer.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !_hasNoteSupplier.get();
    }

}