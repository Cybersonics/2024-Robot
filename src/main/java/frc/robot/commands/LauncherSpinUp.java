package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class LauncherSpinUp extends Command {

    private Launcher _launcher;
    private Supplier<Boolean> _isLauncherUpSupplier;
    private Supplier<Boolean> _isAmpShotSupplier;
    private Supplier<Boolean> _xboxRightTriggerSupplier;
    private Supplier<Boolean> _xboxXButtonSupplier;

    public LauncherSpinUp(Launcher launcher, Supplier<Boolean> isLauncherUpSupplier, Supplier<Boolean> isAmpShotSupplier,
            CommandXboxController xboxController) {
        _launcher = launcher;
        _isLauncherUpSupplier = isLauncherUpSupplier;
        _isAmpShotSupplier = isAmpShotSupplier;

        _xboxRightTriggerSupplier = () -> xboxController.rightTrigger().getAsBoolean();
        _xboxXButtonSupplier = () -> xboxController.x().getAsBoolean();


        addRequirements(_launcher);
    }
    
    public LauncherSpinUp(Launcher launcher, Supplier<Boolean> isLauncherUpSupplier, Supplier<Boolean> isAmpShotSupplier,
            Supplier<Boolean> xboxRightTriggerSupplier, Supplier<Boolean> xboxXButtonSupplier) {
        _launcher = launcher;
        _isLauncherUpSupplier = isLauncherUpSupplier;
        _isAmpShotSupplier = isAmpShotSupplier;
        _xboxRightTriggerSupplier = xboxRightTriggerSupplier;
        _xboxXButtonSupplier = xboxXButtonSupplier;

        addRequirements(_launcher);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (_xboxRightTriggerSupplier.get()) {
            if (_isLauncherUpSupplier.get()) {
                _launcher.setReferenceSpeed(Constants.LauncherConstants.topFarLobRPM, Constants.LauncherConstants.bottomFarLobRPM, _isLauncherUpSupplier.get());
            } else {
                _launcher.setReferenceSpeed(Constants.LauncherConstants.topFarShotRPM, Constants.LauncherConstants.bottomFarShotRPM, _isLauncherUpSupplier.get());
            }
        } else if (_xboxXButtonSupplier.get()) {
            if (_isAmpShotSupplier.get()) {
                _launcher.setReferenceSpeed(Constants.LauncherConstants.topAmpShotRPM, Constants.LauncherConstants.bottomAmpShotRPM, _isLauncherUpSupplier.get());
            } else {
                _launcher.setReferenceSpeed(Constants.LauncherConstants.topSourceLobRPM, Constants.LauncherConstants.bottomSourceLobRPM, _isLauncherUpSupplier.get());
            }
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
