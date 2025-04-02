package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    // Camera names FROM PHOTON
    public static String rightCameraName = "CameraFrontAprilTag";
    public static String leftCameraName = "CameraRight";

    // Camera palcements
    public static Transform3d robotToRightCamera = new Transform3d(0.16, -0.33, 0.25, new Rotation3d(0.0,0, Units.degreesToRadians(30)));
    public static Transform3d robotToLeftCamera = new Transform3d(0.16, 0.33, 0.25, new Rotation3d(0.0, 0, Units.degreesToRadians(-30)));

    public static double maxAmbiguity = 0.3;
    public static double maxZError = 1.0;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.07; // Radians 

    // Standard dev factors for each camera
    public static double[] cameraStdDevFactors = new double[] {
            1.0, 
            1.0
    };
}