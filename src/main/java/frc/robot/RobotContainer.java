// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.NavXGyro;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drive;

public class RobotContainer {
  public static NavXGyro _gyro = NavXGyro.getInstance(); // This must be called before Drive as it is used by the Drive
  public static Drive _drive = Drive.getInstance(_gyro);
  //public static Launcher _launcher = Launcher.getInstance();
  private static Intake _intake = Intake.getInstance();

  public final CommandJoystick leftStick = new CommandJoystick(OperatorConstants.LeftStick);
  public final CommandJoystick rightStick = new CommandJoystick(OperatorConstants.RightStick);

  //public final CommandXboxController xboxController = new CommandXboxController(2);

  public RobotContainer() {

    CommandScheduler.getInstance()
      .setDefaultCommand(_drive, new DriveCommand(_drive, leftStick, rightStick, _gyro));
      //.setDefaultCommand(_drive, new DriveCommand(_drive, xboxController, _gyro));


    configureBindings();
  }

  private void configureBindings() {
    // xboxController.a().onTrue(new InstantCommand(() -> _launcher.setLauncherSpeed(.85)));
    // xboxController.a().onFalse(new InstantCommand(() -> _launcher.setLauncherSpeed(0)));

    // xboxController.b().onTrue(new InstantCommand(() -> _launcher.setFeederSpeed(1)));
    // xboxController.b().onFalse(new InstantCommand(() -> _launcher.setFeederSpeed(0.0)));

    //xboxController.x().onTrue(new InstantCommand(() -> _intake.setIntakeSpeed(1)));
    //xboxController.x().onFalse(new InstantCommand(() -> _intake.setIntakeSpeed(0.0)));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
