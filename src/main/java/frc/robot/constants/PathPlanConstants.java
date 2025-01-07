package frc.robot.constants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.SwerveDriveConstants.ModuleConstants;
import frc.robot.constants.SwerveDriveConstants.TranslationConstants;

public class PathPlanConstants {
    // PathPlanner configuration
    public static final double wheelRadiusMeters = Units.inchesToMeters(2);
    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.47; // Free speed of NEO * kDriveEncoderRot2Meter
    public static final int driveMotorCurrentLimit = 50;
    public static final DCMotor driveGearbox = DCMotor.getNEO(1);


    public static final double robotMassKg = 74.088;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.0;
    
    public static final Translation2d[] moduleTranslations = TranslationConstants.moduleTranslations;
    public static final RobotConfig robotConfig = new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                    wheelRadiusMeters,
                    kPhysicalMaxSpeedMetersPerSecond,
                    wheelCOF,
                    driveGearbox.withReduction(1 / ModuleConstants.kDriveMotorGearRatio),
                    driveMotorCurrentLimit,
                    1),
            moduleTranslations);
}
