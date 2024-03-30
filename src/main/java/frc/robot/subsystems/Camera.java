package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.AprilTag;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class Camera extends SubsystemBase {
    private static Camera instance;
    // private final PhotonCamera camera;
    private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(14.5);
    // Angle between horizontal and the camera.
    private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(32);

    // The field from AprilTagFields will be different depending on the game. only returns pose objects no height
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private double TARGET_HEIGHT_METERS; // Convert to meters when setting

    private double yawVal = 0, pitchVal = 0, skewVal = 0, areaVal = 0;
    private int targetId = 0;
    private boolean hasTarget = false;

    public Camera() { }

    public static Camera getInstance() {
        if (instance == null) {
            instance = new Camera();
        }
        return instance;
    }

    public double getRange() {
        TARGET_HEIGHT_METERS = getTarget().getHeight();
        double range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(getPitchVal()));
        double rangeInInches = Units.metersToInches(range);

        SmartDashboard.putNumber("Camera Distance", rangeInInches);

        return range;
    }

    public double getYawVal() {
        return this.yawVal;
    }

    public double getPitchVal() {
        return this.pitchVal;
    }

    public double getSkewVal() {
        return this.skewVal;
    }

    public double getAreaVal() {
        return this.areaVal;
    }

    public boolean hasTargets() {
        return this.hasTarget;
    }

    public int getTargetId() {
        return this.targetId;
    }

    public AprilTag getTarget() {
        return Constants.AprilTags.AprilTags.get(getTargetId());
    }
}
