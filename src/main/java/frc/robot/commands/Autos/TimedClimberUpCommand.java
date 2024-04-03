package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class TimedClimberUpCommand extends Command {
    private Climber _climber;
    private Timer _timer;
    private double _runTimeInSeconds;

    public TimedClimberUpCommand(Climber climber, double runTimeInSeconds) {
        _climber = climber;
        _runTimeInSeconds = runTimeInSeconds;
        
        addRequirements(_climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _timer = new Timer();
        _timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        _climber.setLeftClimberSpeed(1);
        _climber.setRightClimberSpeed(1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _timer.stop();
        _timer.reset();
        _climber.setLeftClimberSpeed(0);
        _climber.setRightClimberSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return _timer.hasElapsed(_runTimeInSeconds);
    }
}
