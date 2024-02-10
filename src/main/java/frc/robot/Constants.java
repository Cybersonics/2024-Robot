package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Constants {
    public static class OperatorConstants {
        public static final int LeftStick = 0;
        public static final int RightStick = 1;
        public static final int OpController = 2;
        public static final int xboxDriveController = 1;
    }

    public static class AutoConstants {
        public static final double kPhysicalMaxSpeedMetersPerSecond = 6;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

        public static final double kMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;
        public static final double kPXController = 5;
        public static final double kPYController = 5;
        public static final double kPThetaController = 5;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
        public static final double kGoToPointLinearP = 0;
        public static final double kGoToPointLinearF = 0.5;
        public static final double kGoToPointAngularP = 0;
        public static final double kGoToPointAngularF = 0;

        public static final double maxTrajectoryOverrunSeconds = 3;
        public static final double kMaxDistanceMetersError = 0.1;
        public static final double kMaxAngleDegreesError = 5;
    }

    public static class DriveConstants {
        public static final int FrontLeftSteer = 20;
        public static final int FrontLeftDrive = 10;
        public static final int FrontLeftEncoderOffset = 0;

        public static final int FrontRightSteer = 23;
        public static final int FrontRightDrive = 13;
        public static final int FrontRightEncoderOffset = 0;

        public static final int BackLeftSteer = 21;
        public static final int BackLeftDrive = 11;
        public static final int BackLeftEncoderOffset = 0;

        public static final int BackRightSteer = 22;
        public static final int BackRightDrive = 12;
        public static final int BackRightEncoderOffset = 0;

        public static final class ModuleConstants {
            public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
            public static final double kDriveMotorGearRatio = 1 / 6.429;
            public static final double kTurningMotorGearRatio = 1 / 1024;
            public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
            public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
            public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
            public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

            public static final double maxSpeed = 4.5; // M/S

            public static final double kPTurning = 0.5;
        }

        public static final class FrameConstants {
            // Distance between right and left wheels
            public static final double WHEEL_BASE_WIDTH = 21;
            public static final double kTrackWidth = Units.inchesToMeters(WHEEL_BASE_WIDTH);

            // Distance between front and back wheels
            public static final double WHEEL_BASE_LENGTH = 17.25;
            public static final double kWheelBase = Units.inchesToMeters(WHEEL_BASE_LENGTH);

            public static final Translation2d flModuleOffset = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
            public static final Translation2d frModuleOffset = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
            public static final Translation2d blModuleOffset = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
            public static final Translation2d brModuleOffset = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);

            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                    flModuleOffset,
                    frModuleOffset,
                    blModuleOffset,
                    brModuleOffset);

            public static final double kPhysicalMaxSpeedMetersPerSecond = 6;
            public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
        }

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0), // Translation constants
                new PIDConstants(5.0, 0, 0), // Rotation constants
                ModuleConstants.maxSpeed,
                FrameConstants.flModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module)
                new ReplanningConfig());
    }

    public static final class IntakeConstants {
        public static final int intakeMotor = 30;
    }

    public static final class LauncherConstants {
        public static final int launcherTopMotor = 31;
        public static final int launcherBottomMotor = 32;
        public static final int feederMotor = 33;
    }

    public static final class ShuffleBoardConstants {
        private static ShuffleboardTab driveTab = Shuffleboard.getTab("Drive Tab");
    }
}
