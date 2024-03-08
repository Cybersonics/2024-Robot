package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private static Climber instance;
    private CANSparkMax leftClimber;
    private CANSparkMax rightClimber;
    
    public Climber() {
        leftClimber = new CANSparkMax(Constants.ClimberConstants.leftClimber, MotorType.kBrushless);
        leftClimber.restoreFactoryDefaults();
        leftClimber.setIdleMode(IdleMode.kBrake);

        rightClimber = new CANSparkMax(Constants.ClimberConstants.rightClimber, MotorType.kBrushless);
        rightClimber.restoreFactoryDefaults();
        rightClimber.setIdleMode(IdleMode.kBrake);

    }

    public static Climber getInstance() {
        if(instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public void setClimberSpeed(double speed) {
        leftClimber.set(speed);
        rightClimber.set(speed);
    }
}
