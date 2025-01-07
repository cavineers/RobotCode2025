package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class SwerveDriveConstants {
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / (150.0 / 7);
        public static final double kTurningDegreesToRad = Math.PI / 180;
        public static final double kDriveEncoderRot2Rad = kDriveMotorGearRatio * Math.PI * 2 * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2RadPerSec = kDriveEncoderRot2Rad / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    }

    public static final class DriveConstants {

        public static final double kOdometryFrequency = 100.0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.47; // Free speed of NEO * kDriveEncoderRot2Meter
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        public static final double wheelRadiusMeters = Units.inchesToMeters(2);

        public static final int kFrontLeftDriveCanID = 1;
        public static final int kFrontRightDriveCanID = 3;
        public static final int kBackRightDriveCanID = 5;
        public static final int kBackLeftDriveCanID = 7;

        public static final int kFrontLeftTurningCanID = 2;
        public static final int kFrontRightTurningCanID = 4;
        public static final int kBackRightTurningCanID = 6;
        public static final int kBackLeftTurningCanID = 8;

        public static final int kFrontLeftAbsoluteEncoderPort = 9;
        public static final int kFrontRightAbsoluteEncoderPort = 10;
        public static final int kBackRightAbsoluteEncoderPort = 11;
        public static final int kBackLeftAbsoluteEncoderPort = 12;

        public static final int kPigeonID = 23;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kFrontLeftAbsoluteEncoderOffset = 0; 
        public static final double kBackLeftAbsoluteEncoderOffset = 0; 
        public static final double kFrontRightAbsoluteEncoderOffset = 0; 
        public static final double kBackRightAbsoluteEncoderOffset = 0;

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(23.75);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(24.75);
        public static final double kDriveBaseRads = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
    }

    public static final class TranslationConstants {
        public static final Translation2d[] moduleTranslations =
            new Translation2d[] {
                new Translation2d(DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0),
                new Translation2d(DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0),
                new Translation2d(-DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0),
                new Translation2d(-DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0)
            };
        public static final SwerveDriveKinematics SwerveKinematics = new SwerveDriveKinematics(moduleTranslations);
    }
    // Drive PID configuration
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;
    public static final double driveSimP = 1;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.01365;
    public static final double driveSimKv = 0.13395;

    // Turn PID configuration
    public static final double turnKp = 2.0;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 9.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians
}
