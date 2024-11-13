package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LauncherFeeder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;

public class LauncherFeed extends Command {
    private LauncherFeeder _launcherFeeder;
    private CommandJoystick _rightJoystick;
    private CommandJoystick _leftJoystick;
    //private XboxController _driveController;
  private CommandXboxController _driveController = new CommandXboxController(OperatorConstants.xboxDriveController);
    private XboxController driveController_HID = _driveController.getHID();
    private JoystickButton xboxA = new JoystickButton(driveController_HID, XboxController.Button.kA.value);
    private JoystickButton xboxB = new JoystickButton(driveController_HID, XboxController.Button.kRightBumper.value); 
    private JoystickButton xboxY = new JoystickButton(driveController_HID, XboxController.Button.kY.value);

    private Trigger _rightJoystickButtonTwo;
    private Trigger _rightJoyStickTrigger;
    private Trigger _leftJoystickButtonTwo;



    public LauncherFeed(LauncherFeeder launcherFeeder, CommandJoystick righJoystick, CommandJoystick leftJoystick) {
        _rightJoystick = righJoystick;
        _leftJoystick = leftJoystick;
        _launcherFeeder = launcherFeeder;

        _rightJoystickButtonTwo = _rightJoystick.button(2);
        _rightJoyStickTrigger = _rightJoystick.trigger();
        _leftJoystickButtonTwo = _leftJoystick.button(2);
        
        addRequirements(_launcherFeeder);
    }

    public LauncherFeed(LauncherFeeder launcherFeeder, CommandXboxController driveController) {
        _driveController = driveController;
        _launcherFeeder = launcherFeeder;

        _rightJoystickButtonTwo = xboxA;
//        _rightJoystick.button(2);
        _rightJoyStickTrigger = xboxB;
        _leftJoystickButtonTwo = xboxY;
        
        addRequirements(_launcherFeeder);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(_rightJoystickButtonTwo.and(_rightJoyStickTrigger).getAsBoolean()) {
            _launcherFeeder.setFeederSpeed(.35);
        } else if(_rightJoyStickTrigger.getAsBoolean()) {
            _launcherFeeder.setFeederSpeed(1);
        } else if(_leftJoystickButtonTwo.getAsBoolean()) {
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
