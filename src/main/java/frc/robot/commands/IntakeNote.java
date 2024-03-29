package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
    
    private Intake _intake;
    private BooleanSupplier _xboxLeftTriggerSupplier;
    private BooleanSupplier _xboxLeftbumperSupplier;
    
    public IntakeNote(Intake intake, CommandXboxController xboxController) {
        _intake = intake;
        
        _xboxLeftTriggerSupplier = xboxController.leftTrigger();
        _xboxLeftbumperSupplier = xboxController.leftBumper();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(_intake);
    }

    public IntakeNote(Intake intake, BooleanSupplier xboxLeftTriggerSupplier, BooleanSupplier xboxLeftBumSupplier) {
          _intake = intake;
          _xboxLeftTriggerSupplier = xboxLeftTriggerSupplier;
          _xboxLeftbumperSupplier = xboxLeftBumSupplier;

          addRequirements(_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(_xboxLeftTriggerSupplier.getAsBoolean()) {
            _intake.intakeNote(-1, -1, .7);
        } else if(_xboxLeftbumperSupplier.getAsBoolean()) {
            _intake.intakeNote(1, 1, 0);
        } else {
            _intake.intakeNote(0, 0, 0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
