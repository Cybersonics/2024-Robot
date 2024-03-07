package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.BlinkinLEDController;
import frc.robot.subsystems.BlinkinLEDController.BlinkinPattern;

public class LedControlCommand extends Command {

    private Supplier<Boolean> _hasNoteSupplier;
    private BlinkinLEDController _blinkin;

    public LedControlCommand(BlinkinLEDController blinkin, Supplier<Boolean> hasNotSupplier) {
        _hasNoteSupplier = hasNotSupplier;
        _blinkin = blinkin;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(_blinkin);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(_hasNoteSupplier.get()) {
            _blinkin.setPattern(BlinkinPattern.WHITE);
        } else {
            _blinkin.off();
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
