package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private static Intake instance;
    private CANSparkMax intakeMotor;
    private CANSparkMax feederMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotor, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kCoast);

        feederMotor = new CANSparkMax(Constants.IntakeConstants.feederMotor, MotorType.kBrushless);
        feederMotor.restoreFactoryDefaults();
        feederMotor.setIdleMode(IdleMode.kCoast);
        // feederMotor.setInverted(invertDrive);
        // feederMotor.setSmartCurrentLimit(40);
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }    

    public void setFeederSpeed(double speed) {
        feederMotor.set(speed);
    }    
}
