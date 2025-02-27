package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // Camera names FROM PHOTON
    public static String frontCameraName = "Camera1";
    public static String backCameraName = "Camera2";

    // Camera palcements
    public static Transform3d robotToFrontCam = new Transform3d(0.2, 0.0, 0.15, new Rotation3d(0.0,Units.degreesToRadians(-15), 0));
    public static Transform3d robotToBackCam = new Transform3d(0, 0.0, 0, new Rotation3d(0.0, 0, Math.PI));

    public static double maxAmbiguity = 0.3;
    public static double maxZError = 1.0;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard dev factors for each camera
    public static double[] cameraStdDevFactors = new double[] {
            1.0, // front
            1.0 // back
    };
}