package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.utility.shuffleBoardDrive;

public class SwerveModule extends SubsystemBase {

    private CANSparkMax driveMotor;
    private CANSparkMax steerMotor;

    private RelativeEncoder driveMotorEncoder; // Set up integrated Drive motor encoder in Spark Max/Neo
    private RelativeEncoder steerMotorEncoder; // Set up integrated Steer motor encoder in Spark Max/550

    private static final double RAMP_RATE = 0.5;

    public SwerveModule(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer, shuffleBoardDrive driveData) {

        // Create and configure a new Drive motor
        driveMotor = new CANSparkMax(driveNum, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(invertDrive); // setInverted reverses the both the motor and the encoder direction.
        driveMotor.setOpenLoopRampRate(RAMP_RATE); // This provides a motor ramp up time to prevent brown outs.
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(55);        
        driveMotor.burnFlash(); // Set configuration values to flash memory in Spark Max to prevent errors.

        // Create and configure a new Steering motor
        steerMotor = new CANSparkMax(steerNum, MotorType.kBrushless);
        steerMotor.restoreFactoryDefaults();
        steerMotor.setInverted(invertSteer);
        steerMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setSmartCurrentLimit(30);
        steerMotor.burnFlash();

        // Create the built in motor encoders
        driveMotorEncoder = driveMotor.getEncoder();
        driveMotorEncoder.setPositionConversionFactor(DriveConstants.ModuleConstants.kDriveEncoderRot2Meter);
        driveMotorEncoder.setVelocityConversionFactor(DriveConstants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        steerMotorEncoder = steerMotor.getEncoder();
        steerMotor.burnFlash();// Set configuration values to flash memory in Spark Max to prevent errors.
    }
}
