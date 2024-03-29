package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants.ModuleConstants;

public class Climber extends SubsystemBase {
    private static Climber instance;
    private CANSparkMax leftClimber;
    private CANSparkMax rightClimber;
    
    // private RelativeEncoder leftEncoder; // Internal Motor Encoder
    private AbsoluteEncoder leftEncoder; // External Through Bore Encoder
    private SparkPIDController leftPIDController;
    private double leftP = 0.0, leftI = 0.0, leftD = 0.0;

    // private RelativeEncoder rightEncoder; // Internal Motor Encoder
    private AbsoluteEncoder rightEncoder; // External Through Bore Encoder
    private SparkPIDController rightPIDController;
    private double rightP = 0.0, rightI = 0.0, rightD = 0.0;
    
    public Climber() {
        leftClimber = new CANSparkMax(Constants.ClimberConstants.leftClimber, MotorType.kBrushless);
        // leftClimber.restoreFactoryDefaults();
        leftClimber.setIdleMode(IdleMode.kBrake);
        //leftClimber.setInverted(true); // Reverses controller to match up and down but breaks limit sensor.
        leftEncoder = leftClimber.getAbsoluteEncoder(Type.kDutyCycle);
        leftPIDController = leftClimber.getPIDController();
        leftPIDController.setFeedbackDevice(leftEncoder);
        //leftEncoder.setInverted(true);

        leftPIDController.setP(leftP);        
        leftPIDController.setI(leftI);
        leftPIDController.setD(leftD);
        leftPIDController.setOutputRange(-1, 1);

        rightClimber = new CANSparkMax(Constants.ClimberConstants.rightClimber, MotorType.kBrushless);
        // rightClimber.restoreFactoryDefaults();
        rightClimber.setIdleMode(IdleMode.kBrake);
        //rightClimber.setInverted(true); // Reverses controller to match up and down but breaks limit sensor.
        rightEncoder = rightClimber.getAbsoluteEncoder(Type.kDutyCycle);
        rightPIDController = rightClimber.getPIDController();
        rightPIDController.setFeedbackDevice(rightEncoder);
        //rightEncoder.setInverted(true);

        rightPIDController.setP(rightP);        
        rightPIDController.setI(rightI);
        rightPIDController.setD(rightD);
        rightPIDController.setOutputRange(-1, 1);
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

    public void setClimberPosition(double leftPosition, double rightPosition) {
        leftPIDController.setReference(leftPosition, CANSparkMax.ControlType.kPosition);
        rightPIDController.setReference(rightPosition, CANSparkMax.ControlType.kPosition);
    }

    public void setLeftClimberSpeed(double speed) {        
        SmartDashboard.putNumber("Left Climber Position", getLeftEncoderPosition());
        //if(getLeftEncoderPosition() < 200) {
            leftClimber.set(speed);
        //}
    }

    public void setRightClimberSpeed(double speed) {        
        SmartDashboard.putNumber("Right Climber Position", getRightEncoderPosition());
        //if(getRightEncoderPosition() < 270) {
            rightClimber.set(speed);
        //}
    }

    public double getLeftEncoderPosition() {      
        return leftEncoder.getPosition();
    }

    public double getRightEncoderPosition() {
        return rightEncoder.getPosition();
    }
}
