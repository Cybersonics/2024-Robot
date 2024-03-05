package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LauncherSpinUp extends Command {

    private Launcher _launcher;
    private Supplier<Boolean> _isLauncherUpSupplier;
    private CommandXboxController _xboxController;
    private Trigger _xboxRightTrigger;
    private Trigger _xboxXButton;

    public LauncherSpinUp(Launcher launcher, CommandXboxController xboxController) {
        _launcher = launcher;
        _xboxController = xboxController;
        addRequirements(_launcher);
    }
    
    public LauncherSpinUp(Launcher launcher, Supplier<Boolean> isLauncherUpSupplier, CommandXboxController xboxController) {
        _launcher = launcher;
        _isLauncherUpSupplier = isLauncherUpSupplier;
        _xboxController = xboxController;
        addRequirements(_launcher);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _xboxRightTrigger = _xboxController.rightTrigger();
        _xboxXButton = _xboxController.x();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (_xboxRightTrigger.getAsBoolean()) {
            if(_isLauncherUpSupplier.get()) {
                _launcher.setReferenceSpeed(Constants.LauncherConstants.topFarLobRPM, Constants.LauncherConstants.bottomFarLobRPM, _isLauncherUpSupplier.get());
            } else {
                _launcher.setReferenceSpeed(Constants.LauncherConstants.topFarShotRPM, Constants.LauncherConstants.bottomFarShotRPM, _isLauncherUpSupplier.get());
            }
            //_launcher.setReferenceSpeed(_pneumatics.IsLauncherUp());
            //_launcher.setLauncherSpeed(0.83);//Orig 0.75
        } else if (_xboxXButton.getAsBoolean()) {
            _launcher.setReferenceSpeed(Constants.LauncherConstants.topSourceLobRPM, Constants.LauncherConstants.bottomSourceLobRPM, _isLauncherUpSupplier.get());
        } else {
            _launcher.setLauncherSpeed(0);
        }
        
        SmartDashboard.putBoolean("LauncherAtSpeed", _launcher.AtReferenceSpeed());
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
