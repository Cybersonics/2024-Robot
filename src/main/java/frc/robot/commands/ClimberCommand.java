package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command {
    private Climber _climber;

    private DoubleSupplier _xboxLeftJoyStickSupplier;
    private DoubleSupplier _xboxRightJoyStickSupplier;

    public ClimberCommand(Climber climber, DoubleSupplier xboxLeftJoyStickSupplier,
            DoubleSupplier xboxRightJoyStickSupplier) {
        _climber = climber;
        _xboxLeftJoyStickSupplier = xboxRightJoyStickSupplier;
        _xboxRightJoyStickSupplier = xboxLeftJoyStickSupplier;

        addRequirements(_climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double leftJoystickValue = _xboxLeftJoyStickSupplier.getAsDouble();
        if (leftJoystickValue > 0.1 || leftJoystickValue < -0.1) {
            _climber.setLeftClimberSpeed(leftJoystickValue);
        } else {
            _climber.setLeftClimberSpeed(0);
        }

        double rightJoyStickValue = _xboxRightJoyStickSupplier.getAsDouble();
        if (rightJoyStickValue > 0.1 || rightJoyStickValue < -0.1) {
            _climber.setRightClimberSpeed(rightJoyStickValue);
        } else {
            _climber.setRightClimberSpeed(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _climber.setClimberSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
