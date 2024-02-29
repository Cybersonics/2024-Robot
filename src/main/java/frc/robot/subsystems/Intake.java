package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private static Intake instance;
    private CANSparkMax intakeMotor;
    private CANSparkFlex feederMotor;
    private CANSparkMax centeringMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotor, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kCoast);

        feederMotor = new CANSparkFlex(Constants.IntakeConstants.feederMotor, MotorType.kBrushless);
        feederMotor.restoreFactoryDefaults();
        feederMotor.setIdleMode(IdleMode.kCoast);
        // feederMotor.setInverted(invertDrive);
        // feederMotor.setSmartCurrentLimit(40);

        centeringMotor = new CANSparkMax(37, MotorType.kBrushless);
        centeringMotor.restoreFactoryDefaults();
        centeringMotor.setIdleMode(IdleMode.kCoast);
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public void intakeNote(double intakeSpeed, double feederSpeed, double centeringSpeed) {
        intakeMotor.set(intakeSpeed);
        feederMotor.set(feederSpeed);
        centeringMotor.set(centeringSpeed);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }    

    public void setFeederSpeed(double speed) {
        feederMotor.set(speed);
    } 
    
    public void setCenteringSpeed(double speed) {
        centeringMotor.set(speed);
    }
}
