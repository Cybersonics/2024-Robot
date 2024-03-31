// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LauncherFeeder;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.Pneumatics;
import frc.robot.commands.AmpArmExtension;
import frc.robot.commands.CameraAlignment;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.LauncherFeed;
import frc.robot.commands.LauncherSpinUp;
import frc.robot.commands.LedControlCommand;
import frc.robot.commands.Autos.FeedNoteAuto;
import frc.robot.commands.Autos.FireNoteAuto;
import frc.robot.commands.Autos.LowerLauncher;
import frc.robot.commands.Autos.PickupNoteAuto;
import frc.robot.commands.Autos.RunLauncher;
import frc.robot.commands.Autos.RaiseLauncher;
import frc.robot.subsystems.BlinkinLEDController;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  public static NavXGyro _gyro = NavXGyro.getInstance(); // This must be called before Drive as it is used by the Drive
  public static Drive _drive = Drive.getInstance(_gyro);
  public static BlinkinLEDController _blinkinLEDController = BlinkinLEDController.getInstance();
  public static Pneumatics _pneumatics = Pneumatics.getInstance();
  public static Launcher _launcher = Launcher.getInstance(_pneumatics::IsLauncherUp);
  public static LauncherFeeder _launcherFeeder = LauncherFeeder.getInstance();
  public static Intake _intake = Intake.getInstance();
  public static Climber _climber = Climber.getInstance();
  public static Camera _camera = Camera.getInstance();
  public final CommandJoystick leftStick = new CommandJoystick(OperatorConstants.LeftStick);
  public final Joystick leftStick_HID = leftStick.getHID();
  public final CommandJoystick rightStick = new CommandJoystick(OperatorConstants.RightStick);
  public final Joystick righStick_HID = rightStick.getHID();

  public final CommandXboxController xboxController = new CommandXboxController(2);
  public final XboxController xboxController_HID = xboxController.getHID();
  public final JoystickButton xboxA = new JoystickButton(xboxController_HID, XboxController.Button.kA.value);
  public final JoystickButton xboxB = new JoystickButton(xboxController_HID, XboxController.Button.kB.value); 
  public final JoystickButton xboxY = new JoystickButton(xboxController_HID, XboxController.Button.kY.value);


  // Setup Sendable chooser for picking autonomous program in SmartDashboard
  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {

    // Register named commands
    configureNamedCommands();

    CommandScheduler.getInstance()
      .setDefaultCommand(_drive, new DriveCommand(_drive, leftStick, rightStick, _gyro));
    // .setDefaultCommand(_drive, new DriveCommand(_drive, xboxController, _gyro));

    CommandScheduler.getInstance()
        .setDefaultCommand(_launcher, new LauncherSpinUp(_launcher, _pneumatics::IsLauncherUp, _pneumatics::IsArmOut, 
        () -> xboxController_HID.getRightTriggerAxis() > 0.01,
        xboxController_HID::getXButton));
        // .setDefaultCommand(_launcher, new LauncherSpinUp(_launcher, _pneumatics::IsLauncherUp, _pneumatics::IsArmOut, xboxController));

    CommandScheduler.getInstance()
        .setDefaultCommand(_intake, new IntakeNote(_intake, 
          () -> xboxController_HID.getLeftTriggerAxis() > 0.01, 
          xboxController_HID::getLeftBumper));    
        // .setDefaultCommand(_intake, new IntakeNote(_intake, xboxController));


    CommandScheduler.getInstance()
        .setDefaultCommand(_launcherFeeder, new LauncherFeed(_launcherFeeder, rightStick, leftStick));

    CommandScheduler.getInstance()        
    .setDefaultCommand(_climber, new ClimberCommand(_climber, 
      xboxController_HID::getLeftY,
      xboxController_HID::getRightY));

    CommandScheduler.getInstance()
      .setDefaultCommand(_blinkinLEDController, new LedControlCommand(_blinkinLEDController, _intake::topHasNote, _pneumatics::IsLauncherUp));

    // Configure Autonomous Options
    autonomousOptions();

    configureBindings();
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("QuickFireNoteAuto", new FireNoteAuto(_launcher, _launcherFeeder, _pneumatics::IsLauncherUp, _intake::topHasNote));
    NamedCommands.registerCommand("FireNoteAuto", new FireNoteAuto(_launcher, _launcherFeeder, _pneumatics::IsLauncherUp, _intake::topHasNote));
    NamedCommands.registerCommand("PickupNoteAuto", new PickupNoteAuto(_intake));
    NamedCommands.registerCommand("RaiseLauncher", new RaiseLauncher(_pneumatics));  
    NamedCommands.registerCommand("LowerLauncher", new LowerLauncher(_pneumatics));

    NamedCommands.registerCommand("RunLauncher", new RunLauncher(_launcher, _pneumatics::IsLauncherUp));
    NamedCommands.registerCommand("FeedNoteAuto", new FeedNoteAuto(_launcherFeeder, _intake::topHasNote));
  }

  private void configureBindings() {

    xboxA.onTrue(new InstantCommand(() -> _pneumatics.launcherToggle(), _pneumatics));    
    xboxB.onTrue(new AmpArmExtension(_pneumatics, _intake::topHasNote));
    xboxY.onTrue(new CameraAlignment(_camera, _drive, _gyro));
    
    leftStick.button(7).onTrue(new InstantCommand(() -> _gyro.zeroNavHeading(), _gyro));
  }

  public Command getAutonomousCommand() {
    // Get the selected Auto in smartDashboard
    return m_chooser.getSelected();
  }

  /**
   * Use this to set Autonomous options for selection in Smart Dashboard
   */
  private void autonomousOptions() {
    // Example adding Autonomous option to chooser
    m_chooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", m_chooser);

  }
}
