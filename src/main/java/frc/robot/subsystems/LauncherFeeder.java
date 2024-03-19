package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherFeeder extends SubsystemBase {
    private static LauncherFeeder instance;
    private CANSparkFlex feederMotor;

     public LauncherFeeder() {
        feederMotor = new CANSparkFlex(Constants.LauncherConstants.feederMotor, MotorType.kBrushless);
        // feederMotor.restoreFactoryDefaults();
        feederMotor.setIdleMode(IdleMode.kCoast);
        // feederMotor.setInverted(invertDrive);
        // feederMotor.setSmartCurrentLimit(40);
    }
    public static LauncherFeeder getInstance() {
        if (instance == null) {
            instance = new LauncherFeeder();
        }
        return instance;
    }

    public void setFeederSpeed(double speed) {
        feederMotor.set(speed);
    }    
}
