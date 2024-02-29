// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
//import frc.robot.commands.IntakeNoteAuto;
import frc.robot.commands.LauncherFeed;
import frc.robot.commands.LauncherSpinUp;
import frc.robot.commands.Autos.FeedNoteAuto;
import frc.robot.commands.Autos.IntakeNoteAuto;
import frc.robot.commands.Autos.LauncherSpinUpAuto;
import frc.robot.commands.Autos.RaiseLauncher;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;

import javax.management.InstanceAlreadyExistsException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

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
        .setDefaultCommand(_launcher, new LauncherSpinUp(_launcher, xboxController));

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
    NamedCommands.registerCommand("LauncherSpinUpAuto", new LauncherSpinUpAuto(_launcher));
    NamedCommands.registerCommand("RaiseLauncher", new RaiseLauncher(_pneumatics));
    NamedCommands.registerCommand("FeedNoteAuto", new FeedNoteAuto(_launcherFeeder));
    NamedCommands.registerCommand("IntakeNote", new IntakeNoteAuto(_intake));
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
