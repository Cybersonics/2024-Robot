package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
    
    private Intake _intake;
    private CommandXboxController _xboxController;

    public IntakeNote(Intake intake, CommandXboxController xboxController) {
        _intake = intake;
        _xboxController = xboxController;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(_xboxController.leftTrigger().getAsBoolean()) {
            _intake.intakeNote(-1, -0.7);
        } else {
            _intake.intakeNote(0, 0);
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
