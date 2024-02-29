package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;

public class IntakeNoteAuto extends Command {
    
    private Intake _intake;
    private Timer _timer;

    public IntakeNoteAuto(Intake intake) {
        _intake = intake;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(_intake);
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
        _intake.intakeNote(-1, -0.7, .5);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _intake.intakeNote(0, 0, 0);
        _timer.stop();
        _timer.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return _timer.hasElapsed(1);
    }
}
