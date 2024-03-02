package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
    private static Launcher instance;
    private CANSparkFlex topMotor;
    private CANSparkFlex bottomMotor;
    private RelativeEncoder topEncoder;
    private RelativeEncoder bottomEncoder;
    private SparkPIDController topPIDController;
    private SparkPIDController bottomPIDController;
    
  public double topKP, topKI, topKD, topKIz, farTopKFF, closeTopKFF, topKMaxOutput, topMinOutput, topMaxRPM;  
  public double bottomKP, bottomKI, bottomKD, bottomKIz, farBottomKFF, closeBottomKFF, bottomMaxOutput, bottomMinOutput, bottomMaxRPM;


    public Launcher() {
        topMotor = new CANSparkFlex(Constants.LauncherConstants.launcherTopMotor, MotorType.kBrushless);
        topMotor.restoreFactoryDefaults();
        topMotor.setIdleMode(IdleMode.kCoast);
        topEncoder = topMotor.getEncoder();
        topPIDController = topMotor.getPIDController();
        // motorOne.setInverted(invertDrive);
        // motorOne.setSmartCurrentLimit(40);

        bottomMotor = new CANSparkFlex(Constants.LauncherConstants.launcherBottomMotor, MotorType.kBrushless);
        bottomMotor.restoreFactoryDefaults();
        bottomMotor.setIdleMode(IdleMode.kCoast);
        bottomMotor.setInverted(true);
        bottomEncoder = bottomMotor.getEncoder();
        bottomPIDController = bottomMotor.getPIDController();
        // motorTwo.setInverted(invertDrive);
        // motorTwo.setSmartCurrentLimit(40);

        topKP = 0.001; 
        topKI = 0;//0.00000001;
        topKD = 0; //0.000001; 
        topKIz = 0; 
        farTopKFF = 0.00015;
        closeTopKFF = 0.00005;
        topKMaxOutput = 1; 
        topMinOutput = -1;
        topMaxRPM = 6000;

        bottomKP = 0.0000001; 
        bottomKI = 0;//0.00000001;
        bottomKD = 0; //0.000001; 
        bottomKIz = 0; 
        farBottomKFF = 0.00015;
        closeBottomKFF = 0.00005;

        bottomMaxOutput = 1; 
        bottomMinOutput = -1;
        bottomMaxRPM = 6000;

        topPIDController.setP(topKP);
        topPIDController.setI(topKI);
        topPIDController.setD(topKD);
        topPIDController.setIZone(topKIz);
        topPIDController.setFF(farTopKFF);
        topPIDController.setOutputRange(topMinOutput, topKMaxOutput);

        bottomPIDController.setP(bottomKP);
        bottomPIDController.setI(bottomKI);
        bottomPIDController.setD(bottomKD);
        bottomPIDController.setIZone(bottomKIz);
        bottomPIDController.setFF(farBottomKFF);
        bottomPIDController.setOutputRange(bottomMinOutput, bottomMaxOutput);

        SmartDashboard.putNumber("Input TopShooterRPM", Constants.LauncherConstants.longTopMotor);        
        SmartDashboard.putNumber("Input BottomShooterRPM", Constants.LauncherConstants.longBottomMotor);
    }
    
    public void setReferenceSpeed() {
        boolean isLauncherUp = SmartDashboard.getBoolean("LauncherUp", true);
        double setTopReferenceSpeed = SmartDashboard.getNumber("Input TopShooterRPM", Constants.LauncherConstants.longTopMotor);        
        double setBottomReferenceSpeed = SmartDashboard.getNumber("Input BottomShooterRPM", Constants.LauncherConstants.longBottomMotor);
        if(!isLauncherUp) {
            topPIDController.setFF(closeTopKFF);
            bottomPIDController.setFF(closeBottomKFF);
        } else {
            topPIDController.setFF(farTopKFF);
            bottomPIDController.setFF(farBottomKFF);
        }

        topPIDController.setReference(setTopReferenceSpeed, CANSparkMax.ControlType.kVelocity);        
        bottomPIDController.setReference(setBottomReferenceSpeed, CANSparkMax.ControlType.kVelocity);

        SmartDashboard.putNumber("Reading TopShooterRPM", topEncoder.getVelocity());        
        SmartDashboard.putNumber("Reading BottomShooterRPM", bottomEncoder.getVelocity());
    }

    public static Launcher getInstance() {
        if (instance == null) {
            instance = new Launcher();
        }
        return instance;
    }
    
     
    public void setLauncherSpeed(double speed) {
        topMotor.set(speed);
        bottomMotor.set(speed);
    }

    //public void setShoot(double speed) {
        //if(speed == 0) {
            //topMotor.set(speed);
            //bottomMotor.set(-speed);
        //} else {
            //topMotor.set(0);
            //bottomMotor.set(0);
        //}

    //}
}
