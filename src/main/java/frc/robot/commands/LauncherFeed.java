package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherFeeder;

public class LauncherFeed extends Command {
    private LauncherFeeder _launcherFeeder;

    public LauncherFeed(LauncherFeeder launcherFeeder) {
        _launcherFeeder = launcherFeeder;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        _launcherFeeder.setFeederSpeed(1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _launcherFeeder.setFeederSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
