// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class RobotContainer {
  public static Launcher _launcher = Launcher.getInstance();
  private static Intake _intake = Intake.getInstance();

  public final CommandXboxController xboxController = new CommandXboxController(2);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    xboxController.a().onTrue(new InstantCommand(() -> _launcher.setLauncherSpeed(.85)));
    xboxController.a().onFalse(new InstantCommand(() -> _launcher.setLauncherSpeed(0)));

    xboxController.b().onTrue(new InstantCommand(() -> _launcher.setFeederSpeed(1)));
    xboxController.b().onFalse(new InstantCommand(() -> _launcher.setFeederSpeed(0.0)));

    xboxController.x().onTrue(new InstantCommand(() -> _intake.setIntakeSpeed(1)));
    xboxController.x().onFalse(new InstantCommand(() -> _intake.setIntakeSpeed(0.0)));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
