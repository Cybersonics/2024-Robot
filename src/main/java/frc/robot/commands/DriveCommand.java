// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavXGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveCommand extends Command {

  private Drive _drive;
  private CommandJoystick leftStick;
  private CommandJoystick rightStick;
  private CommandXboxController xboxController;

  private NavXGyro _navXGyro;

  public static final double OMEGA_SCALE = 1.0 / 20.0;//30.0;// 45
  public static final double DEADZONE_LSTICK = 0.1;
  private static final double DEADZONE_RSTICK = 0.1;
  private double originHeading = 0.0;
  private double leftPow = 1;
  private double rightPow = 1;

  /**
   * Creates a new DriveCommand using a standard set of joysticks as the driver
   * joysticks.
   */
  public DriveCommand(Drive drive, CommandJoystick leftStick, CommandJoystick rightStick, NavXGyro gyro) {
    this._drive = drive;
    this.leftStick = leftStick;
    this.rightStick = rightStick;
    this._navXGyro = gyro;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  public DriveCommand(Drive drive, CommandXboxController xboxController, NavXGyro gyro) {
    this._drive = drive;
    this.xboxController = xboxController;
    this._navXGyro = gyro;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*
     * Get the starting position of the gyro.
     * This will be used as the initial angle of the robot for field centric
     * control.
     */
    originHeading = _navXGyro.getZeroAngle();
    // _drive.setDrivesMode(IdleMode.kCoast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // final double originOffset = 360 - originHeading;
    // originCorr = _navXGyro.getNavAngle() + originOffset;

    /*
     * Original stick commands used in original code are commented out below.
     * The sticks are being inverted in the following lines to work with the
     * revised drive code. The revised drive code sets aligns the drive axis
     * and drive directions to match the odd directions of wpilib. This is done
     * to allow the use of path planning software in autonoumous mode.
     */

    // double stickForward = this.driveController.getLeftY();
    // double stickStrafe = this.driveController.getLeftX();
    // double stickOmega = (this.driveController.getRightX());

    double stickForward;
    double stickStrafe;
    double stickOmega;

    if (xboxController != null) {
      stickForward = -this.xboxController.getLeftY() * 0.25;
      stickStrafe = -this.xboxController.getLeftX() * 0.25;
      stickOmega = -this.xboxController.getRightX() * 0.25;
      // stickForward = -this.xboxController.getLeftY();
      // stickStrafe = -this.xboxController.getLeftX();
      // stickOmega = -this.xboxController.getRightX();
    } else {
      stickForward = -this.leftStick.getY()*.9;
      stickStrafe = -this.leftStick.getX()*.9;
      stickOmega = -this.rightStick.getX()*.9;
      // stickForward = this.leftStick.getY();
      // stickStrafe = this.leftStick.getX();
      // stickOmega = this.rightStick.getX();
    }

    // SmartDashboard.putNumber("Controller Forward", stickForward);
    // SmartDashboard.putNumber("Controller Strafe", stickStrafe);
    // SmartDashboard.putNumber("Controller Omega", stickOmega);

    /*
     * The following lines allow the programmer to increase or decrease the initial
     * joystick action. The input of the joystick is taken to a power. If the
     * exponent is
     * one then the joystick action is linear. If the power is two then the initial
     * action will be a "soft" ramp, while the ending action will sharply increase.
     */

    double strafe = Math.pow(Math.abs(stickStrafe), leftPow) * Math.signum(stickStrafe);
    double forward = Math.pow(Math.abs(stickForward), leftPow) * Math.signum(stickForward);
    double omega = Math.pow(Math.abs(stickOmega), rightPow) * Math.signum(stickOmega) * OMEGA_SCALE;

    /*
     * If the input from the joystick is less than a dead zone value then set the
     * joystick output to zero. This prevents the robot from drifting due to the
     * joysticks
     * not fully returning to the zero position.
     * Note: take care when setting the deadzone value. If the value is set to a
     * high value,
     * the robot will move aggresively when the stick goes past the deadzone value.
     */
    if (Math.abs(strafe) < DEADZONE_LSTICK)
      strafe = 0.0;
    if (Math.abs(forward) < DEADZONE_LSTICK)
      forward = 0.0;
    if (Math.abs(omega) < DEADZONE_RSTICK * OMEGA_SCALE)
      omega = 0.0;
    boolean stickFieldCentric;
    if(xboxController != null) {
      stickFieldCentric = xboxController.leftBumper().getAsBoolean();
    }else {
      stickFieldCentric = leftStick.trigger().getAsBoolean();
    }

    if (!stickFieldCentric) {

      /*
       * When the Left Joystick trigger is not pressed, The robot is in Field Centric
       * Mode.
       * The calculations correct the forward and strafe values for field centric
       * attitude.
       * Rotate the velocity vector from the joystick by the difference between our
       * current orientation and the current origin heading.
       */

      /*
       * Get the current angle of the robot and subtract it from the original heading
       * to determine
       * the angle of the robot to the field.
       */
      final double originCorrection = Math.toRadians(originHeading - _navXGyro.getNavAngle());
  
      /*
       * Field centric code only affects the forward and strafe action, not rotation.
       * To perform field
       * centric movements we need to correct the forward and strafe action from the
       * joysticks by using
       * trig to determine how much each joystick movement contributes to moving in
       * the x and y axis of
       * the robot.
       */

      final double temp = forward * Math.cos(originCorrection) + strafe * Math.sin(originCorrection);
      strafe = strafe * Math.cos(originCorrection) - forward * Math.sin(originCorrection);
      forward = temp;
    }

    /*
     * If all of the joysticks are in the deadzone, don't update the motors
     */

    boolean deadStick = false;
    if (strafe == 0.0 && forward == 0.0 && omega == 0.0) {
      deadStick = true;
    }

    // SmartDashboard.putNumber("Forward Done", forward);
    // SmartDashboard.putNumber("Strafe Done", strafe);
    // SmartDashboard.putNumber("Rotation Done", omega);
    SmartDashboard.putBoolean("Dead Stick", deadStick);

    /*
     * Take the calculated values from the joysticks and use the values to operate
     * the drive system.
     */
    this._drive.processInput(forward, strafe, omega, deadStick);
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