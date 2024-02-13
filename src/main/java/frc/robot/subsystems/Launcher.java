package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
    private static Launcher instance;
    private CANSparkFlex topMotor;
    private CANSparkFlex bottomMotor;
    private CANSparkFlex feederMotor;

    public Launcher() {
        topMotor = new CANSparkFlex(Constants.LauncherConstants.launcherTopMotor, MotorType.kBrushless);
        topMotor.restoreFactoryDefaults();
        topMotor.setIdleMode(IdleMode.kCoast);
        // motorOne.setInverted(invertDrive);
        // motorOne.setSmartCurrentLimit(40);

        bottomMotor = new CANSparkFlex(Constants.LauncherConstants.launcherBottomMotor, MotorType.kBrushless);
        bottomMotor.restoreFactoryDefaults();
        bottomMotor.setIdleMode(IdleMode.kCoast);
        // motorTwo.setInverted(invertDrive);
        // motorTwo.setSmartCurrentLimit(40);

        feederMotor = new CANSparkFlex(Constants.LauncherConstants.feederMotor, MotorType.kBrushless);
        feederMotor.restoreFactoryDefaults();
        feederMotor.setIdleMode(IdleMode.kCoast);
        // feederMotor.setInverted(invertDrive);
        // feederMotor.setSmartCurrentLimit(40);
    }

    public static Launcher getInstance() {
        if (instance == null) {
            instance = new Launcher();
        }
        return instance;
    }

    public void setLauncherSpeed(double speed) {
        topMotor.set(speed);
        bottomMotor.set(-speed);
    }

    public void setFeederSpeed(double speed) {
        feederMotor.set(speed);
    }    
}
