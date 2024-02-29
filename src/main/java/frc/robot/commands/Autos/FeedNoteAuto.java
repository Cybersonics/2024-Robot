package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.LauncherFeeder;

public class FeedNoteAuto extends Command {
    private LauncherFeeder _launcherFeeder;
    private Timer _timer;

     public FeedNoteAuto(LauncherFeeder launcherFeeder) {
        _launcherFeeder = launcherFeeder;
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
        _launcherFeeder.setFeederSpeed(1);
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
        return _timer.hasElapsed(.4);
    }

}
