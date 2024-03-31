package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drive;
import frc.robot.utility.AprilTag;
import frc.robot.utility.LimelightHelpers;
import frc.robot.subsystems.NavXGyro;

public class CameraAlignment extends Command {
    private Camera _camera;
    private Drive _drive;
    private NavXGyro _navXGyro;

    private PIDController _driveRotationPID, _driveDistancePID, _driveStrafePID;
    private double _driveRotationP = 0.00035, _driveRotationD = 0.00, _driveRotationI = 0.00;//p=0.0002 p =0.0004 d=0.01
    private double _driveDistanceP = 0.015, _driveDistanceD = 0.008, _driveDistanceI = 0.00;//p=0.002
    private double _driveStrafeP = 0.02, _driveStrafeD = 0.00, _driveStrafeI = 0.00;
    private AprilTag _target;
    private double _aprilTagID;

    public CameraAlignment(Camera camera, Drive drive, NavXGyro gyro) {
        _camera = camera;
        _drive = drive;
        _navXGyro = gyro;

        addRequirements(_camera, _drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { 
        _driveRotationPID = new PIDController(_driveRotationP, _driveRotationI, _driveRotationD);
        _driveRotationPID.setTolerance(1.0);//0.8
        
        _driveDistancePID = new PIDController(_driveDistanceP, _driveDistanceI, _driveDistanceD);
        _driveDistancePID.setTolerance(2);

        _driveStrafePID = new PIDController(_driveStrafeP, _driveStrafeI, _driveStrafeD);
        _driveStrafePID.setTolerance(0.5);//2
        
        _aprilTagID = LimelightHelpers.getFiducialID("");
        //_target = Constants.AprilTags.AprilTags.get(((int)LimelightHelpers.getFiducialID("")-1)); // indexed list is 0-15 not 1-16
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (_aprilTagID>-1){
            _target = Constants.AprilTags.AprilTags.get(((int)_aprilTagID-1)); // indexed list is 0-15 not 1-16
            double targetDistance = _target.getDistance();
            double targetHeight = _target.getHeight();
            double targetHeading = _target.getExpectedHeading();

            SmartDashboard.putNumber("Target Distance", targetDistance);
            SmartDashboard.putNumber("Target Height", targetHeight);
            SmartDashboard.putNumber("Target Heading", targetHeading);
        
        
            //double rotationEstimate = LimelightHelpers.getTY("");// + Constants.TrapConstants.AngleOffset;
            //double rotationEstimate = LimelightHelpers.getTX("");// + Constants.TrapConstants.AngleOffset;
            //double rotationValue = _driveRotationPID.calculate(rotationEstimate, 0);
            double rotationValue = _driveRotationPID.calculate(-_navXGyro.getNavAngle(), targetHeading);
            
            //SmartDashboard.putNumber("Rotation Estimate", rotationEstimate);
            SmartDashboard.putNumber("Rotation Value", rotationValue);


            //How many degrees back is limelight rotated from vertical
            /*Vertical angle calculated by setting bot a fixed distance back from target (measureDistanceToTarget) with 
            height of target and height of camera lens measured in inches.
            Use a calculator to get the Total Angle = arcTan(targetHeight-cameraHeight)/measuredDistanceToTaget
            Using the Limelight webviewer get the ty value. Take the total angle calculated above and subtract the 
            ty value from the Limelight webvier to get the limelightMountAngleDegrees.
            */

            double tx = LimelightHelpers.getTX("");
            //Vertical angle of target in view in degrees
            double ty = LimelightHelpers.getTY(""); 

            double limelightMountAngleDegrees = 29.085;//32; 

            //Distance from center of limelight lens to floor
            double limelightLensHeightInches = 14.5;

            //Distance fron target to floor
            //double goalHeightInches = 52.0;//50.5;

            double angleToGoalDegrees = limelightMountAngleDegrees + ty;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            //Calculate distance
            double distanceFromLimelight = (targetHeight - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

            SmartDashboard.putNumber("Distance Value", distanceFromLimelight);

            double distanceValue = _driveDistancePID.calculate(distanceFromLimelight, targetDistance);

            double strafeValue = _driveStrafePID.calculate(tx, 0);

            // This isn't correct either since it doesn't account for the vector movement its only forward/reverse 
            // and its at the same time as rotaiton maybe we should separate it?     
            // _drive.processInput(distanceValue, 0.0, -rotationValue, false);

            _drive.processInput(distanceValue, -strafeValue, rotationValue, false); //-rotationValue

        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _drive.processInput(0,0,0,true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (_driveRotationPID.atSetpoint()); // && _driveDistancePID.atSetpoint());
    }
}
