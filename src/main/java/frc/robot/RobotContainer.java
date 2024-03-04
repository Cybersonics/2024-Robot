// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LauncherFeeder;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.Pneumatics;
import frc.robot.commands.AmpShot;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.LauncherFeed;
import frc.robot.commands.LauncherSpinUp;
import frc.robot.commands.Autos.FeedNoteAuto;
import frc.robot.commands.Autos.FireNoteAuto;
import frc.robot.commands.Autos.LowerLauncher;
import frc.robot.commands.Autos.PickupNoteAuto;
import frc.robot.commands.Autos.PrespinupLauncher;
import frc.robot.commands.Autos.RaiseLauncher;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  public static NavXGyro _gyro = NavXGyro.getInstance(); // This must be called before Drive as it is used by the Drive
  public static Drive _drive = Drive.getInstance(_gyro);
  public static Launcher _launcher = Launcher.getInstance();
  public static LauncherFeeder _launcherFeeder = LauncherFeeder.getInstance();
  public static Intake _intake = Intake.getInstance();
  public static Pneumatics _pneumatics = Pneumatics.getInstance();
  public static Climber _climber = Climber.getInstance();
  public final CommandJoystick leftStick = new CommandJoystick(OperatorConstants.LeftStick);
  public final CommandJoystick rightStick = new CommandJoystick(OperatorConstants.RightStick);

  public final CommandXboxController xboxController = new CommandXboxController(2);

  // Setup Sendable chooser for picking autonomous program in SmartDashboard
  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {

    // Register named commands
    configureNamedCommands();

    CommandScheduler.getInstance()
        .setDefaultCommand(_drive, new DriveCommand(_drive, leftStick, rightStick, _gyro));
    // .setDefaultCommand(_drive, new DriveCommand(_drive, xboxController, _gyro));

    CommandScheduler.getInstance()
        .setDefaultCommand(_launcher, new LauncherSpinUp(_launcher, _pneumatics::IsLauncherUp, xboxController));

    CommandScheduler.getInstance()
        .setDefaultCommand(_intake, new IntakeNote(_intake, xboxController));

    CommandScheduler.getInstance()
        .setDefaultCommand(_launcherFeeder, new LauncherFeed(_launcherFeeder, rightStick, leftStick));

    CommandScheduler.getInstance()
        .setDefaultCommand(_climber, new ClimberCommand(_climber, xboxController));

    // CommandScheduler.getInstance()
    //     .setDefaultCommand(_launcher, new AmpShot(_launcher, _launcherFeeder, xboxController));

    // Configure Autonomous Options
    autonomousOptions();

    configureBindings();
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("FireNoteAuto", new FireNoteAuto(_launcher, _launcherFeeder, _pneumatics::IsLauncherUp, _intake::hasNote));
    NamedCommands.registerCommand("PickupNoteAuto", new PickupNoteAuto(_intake));
    NamedCommands.registerCommand("RaiseLauncher", new RaiseLauncher(_pneumatics));  
    NamedCommands.registerCommand("LowerLauncher", new LowerLauncher(_pneumatics));

    NamedCommands.registerCommand("PreSpinLauncher", new PrespinupLauncher(_launcher, _pneumatics::IsLauncherUp));
    NamedCommands.registerCommand("FeedNoteAuto", new FeedNoteAuto(_launcherFeeder, _pneumatics::IsLauncherUp));
  }

  private void configureBindings() {
    xboxController.a().onTrue(new InstantCommand(() -> _pneumatics.launcherToggle(), _pneumatics));

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
