package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
    
    private Intake _intake;

    public IntakeNote(Intake intake) {
        _intake = intake;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        _intake.intakeNote(1, .7);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _intake.intakeNote(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
