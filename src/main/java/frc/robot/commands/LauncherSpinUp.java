package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class LauncherSpinUp extends Command {

    private Launcher _launcher;

    public LauncherSpinUp(Launcher launcher) {
        _launcher = launcher;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        _launcher.setLauncherSpeed(.83);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("ending spin up");
        _launcher.setLauncherSpeed(0);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
