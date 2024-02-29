package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.LauncherFeeder;

public class LauncherFeed extends Command {
    private LauncherFeeder _launcherFeeder;
    private CommandJoystick _rightJoystick;
    private CommandJoystick _leftJoystick;


    public LauncherFeed(LauncherFeeder launcherFeeder, CommandJoystick righJoystick, CommandJoystick leftJoystick) {
        _rightJoystick = righJoystick;
        _leftJoystick = leftJoystick;
        _launcherFeeder = launcherFeeder;

        addRequirements(_launcherFeeder);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(_rightJoystick.button(2).and(_rightJoystick.trigger()).getAsBoolean()) {
            _launcherFeeder.setFeederSpeed(.35);
        } else if(_rightJoystick.trigger().getAsBoolean()) {
            _launcherFeeder.setFeederSpeed(1);
        } else if(_leftJoystick.button(2).getAsBoolean()) {
            _launcherFeeder.setFeederSpeed(-.5);
        } else {
            _launcherFeeder.setFeederSpeed(0);
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
