package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class LauncherSpinUp extends Command {

    private Launcher _launcher;
    private BooleanSupplier _isLauncherUpSupplier;
    private BooleanSupplier _isAmpShotSupplier;
    private BooleanSupplier _xboxRightTriggerSupplier;
    private BooleanSupplier _xboxXButtonSupplier;

    public LauncherSpinUp(Launcher launcher, BooleanSupplier isLauncherUpSupplier, BooleanSupplier isAmpShotSupplier,
            CommandXboxController xboxController) {
        _launcher = launcher;
        _isLauncherUpSupplier = isLauncherUpSupplier;
        _isAmpShotSupplier = isAmpShotSupplier;

        _xboxRightTriggerSupplier = xboxController.rightTrigger();
        _xboxXButtonSupplier = xboxController.x();


        addRequirements(_launcher);
    }
    
    public LauncherSpinUp(Launcher launcher, BooleanSupplier isLauncherUpSupplier, BooleanSupplier isAmpShotSupplier,
            BooleanSupplier xboxRightTriggerSupplier, BooleanSupplier xboxXButtonSupplier) {
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
        if (_xboxRightTriggerSupplier.getAsBoolean()) {
            if (_isLauncherUpSupplier.getAsBoolean()) {
                _launcher.setReferenceSpeed(Constants.LauncherConstants.topFarLobRPM, Constants.LauncherConstants.bottomFarLobRPM, _isLauncherUpSupplier.getAsBoolean());
            } else {
                _launcher.setReferenceSpeed(Constants.LauncherConstants.topFarShotRPM, Constants.LauncherConstants.bottomFarShotRPM, _isLauncherUpSupplier.getAsBoolean());
            }
        } else if (_xboxXButtonSupplier.getAsBoolean()) {
            if (_isAmpShotSupplier.getAsBoolean()) {
                _launcher.setReferenceSpeed(Constants.LauncherConstants.topAmpShotRPM, Constants.LauncherConstants.bottomAmpShotRPM, _isLauncherUpSupplier.getAsBoolean());
            } else {
                _launcher.setReferenceSpeed(Constants.LauncherConstants.topSourceLobRPM, Constants.LauncherConstants.bottomSourceLobRPM, _isLauncherUpSupplier.getAsBoolean());
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
