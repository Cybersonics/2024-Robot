package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drive;
import frc.robot.utility.AprilTag;
import frc.robot.utility.LimelightHelpers;

public class CameraAlignment extends Command {
    private Camera _camera;
    private Drive _drive;

    private PIDController _driveRotationPID, _driveDistancePID;
    private double _driveRotationP = 0.0002, _driveRotationD = 0.00, _driveRotationI = 0.00;
    private double _driveDistanceP = 0.002, _driveDistanceD = 0.00, _driveDistanceI = 0.00;
    private AprilTag _target;

    public CameraAlignment(Camera camera, Drive drive) {
        _camera = camera;
        _drive = drive;

        addRequirements(_camera, _drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { 
        _driveRotationPID = new PIDController(_driveRotationP, _driveRotationI, _driveRotationD);
        _driveRotationPID.setTolerance(2);
        
        _driveDistancePID = new PIDController(_driveDistanceP, _driveDistanceI, _driveDistanceD);
        _driveDistancePID.setTolerance(5);
        
        _target = Constants.AprilTags.AprilTags.get(((int)LimelightHelpers.getFiducialID("")-1)); // indexed list is 0-15 not 1-16
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double rotationEstimate = LimelightHelpers.getTY("");// + Constants.TrapConstants.AngleOffset;
        SmartDashboard.putNumber("Rotation Estimate", rotationEstimate);
        double rotationValue = _driveRotationPID.calculate(rotationEstimate, 0);
        SmartDashboard.putNumber("Rotation Value", rotationValue);

        // This is wrong we need to calculate the distance to the target it doesn't seem to be returned.
        //double distanceValue = _driveDistancePID.calculate(LimelightHelpers.getTA(""), _target.getDistance());

        // This isn't correct either since it doesn't account for the vector movement its only forward/reverse 
        // and its at the same time as rotaiton maybe we should separate it?     
        // _drive.processInput(distanceValue, 0.0, -rotationValue, false);

        _drive.processInput(0.0, 0.0, -rotationValue, false); 

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
