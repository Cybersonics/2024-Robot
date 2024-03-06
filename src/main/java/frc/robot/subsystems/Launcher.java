package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
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
    
  public double topKP, topKI, topKD, topKIz, downTopKFF, upTopKFF, topKMaxOutput, topMinOutput, topMaxRPM;  
  public double bottomKP, bottomKI, bottomKD, bottomKIz, downBottomKFF, upBottomKFF, bottomMaxOutput, bottomMinOutput, bottomMaxRPM;


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

        topKP = 0.0002; 
        topKI = 0;//0.00000001;
        topKD = 0; //0.000001; 
        topKIz = 0; 
        downTopKFF = 0.000155;
        upTopKFF = 0.00015;
        topKMaxOutput = 1; 
        topMinOutput = -1;
        topMaxRPM = 6000;

        bottomKP = 0.0000002; 
        bottomKI = 0;//0.00000001;
        bottomKD = 0; //0.000001; 
        bottomKIz = 0; 
        downBottomKFF = 0.000155;
        upBottomKFF = 0.00015;

        bottomMaxOutput = 1; 
        bottomMinOutput = -1;
        bottomMaxRPM = 6000;

        topPIDController.setP(topKP);
        topPIDController.setI(topKI);
        topPIDController.setD(topKD);
        topPIDController.setIZone(topKIz);
        topPIDController.setFF(downTopKFF);
        topPIDController.setOutputRange(topMinOutput, topKMaxOutput);

        bottomPIDController.setP(bottomKP);
        bottomPIDController.setI(bottomKI);
        bottomPIDController.setD(bottomKD);
        bottomPIDController.setIZone(bottomKIz);
        bottomPIDController.setFF(downBottomKFF);
        bottomPIDController.setOutputRange(bottomMinOutput, bottomMaxOutput);

        SmartDashboard.putNumber("Input TopShooterRPM", Constants.LauncherConstants.topFarShotRPM);        
        SmartDashboard.putNumber("Input BottomShooterRPM", Constants.LauncherConstants.bottomFarShotRPM);
    }

    public void setReferenceSpeed(double topMotorTargetVelocity, double bottomMotorTargetVelocity, boolean isLauncherUp) {
        SmartDashboard.putNumber("Input TopShooterRPM", topMotorTargetVelocity);        
        SmartDashboard.putNumber("Input BottomShooterRPM", bottomMotorTargetVelocity);
       if(isLauncherUp) {
            topPIDController.setFF(upTopKFF);
            bottomPIDController.setFF(upBottomKFF);
        } else {
            topPIDController.setFF(downTopKFF);
            bottomPIDController.setFF(downBottomKFF);
        }

        topPIDController.setReference(topMotorTargetVelocity, CANSparkMax.ControlType.kVelocity);        
        bottomPIDController.setReference(bottomMotorTargetVelocity, CANSparkMax.ControlType.kVelocity);
        
        SmartDashboard.putNumber("Reading TopShooterRPM", topEncoder.getVelocity());        
        SmartDashboard.putNumber("Reading BottomShooterRPM", bottomEncoder.getVelocity());
    }
    
    public void setReferenceSpeed(boolean isLauncherUp) {
        double setTopReferenceSpeed = SmartDashboard.getNumber("Input TopShooterRPM", Constants.LauncherConstants.topFarShotRPM);        
        double setBottomReferenceSpeed = SmartDashboard.getNumber("Input BottomShooterRPM", Constants.LauncherConstants.bottomFarShotRPM);
        if(isLauncherUp) {
            topPIDController.setFF(upTopKFF);
            bottomPIDController.setFF(upBottomKFF);
        } else {
            topPIDController.setFF(downTopKFF);
            bottomPIDController.setFF(downBottomKFF);
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

    public boolean AtReferenceSpeed() {
        var tolerance = 375;
        var topTarget = SmartDashboard.getNumber("Input TopShooterRPM", 0);
        var bottomTarget = SmartDashboard.getNumber("Input BottomShooterRPM", 0);

        boolean topOk = MathUtil.isNear(topTarget, topEncoder.getVelocity(), tolerance);
        boolean bottomOk = MathUtil.isNear(bottomTarget, bottomEncoder.getVelocity(), tolerance);
      
        return topOk && bottomOk;
    }
}
