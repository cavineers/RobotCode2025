package frc.robot.subsystems.CanRangeArray;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class CanRangeArrayConstants {

    public static double kDifferenceTolerance = 0.1; // Meters -> difference between two sensors to be considered the same
    public static double kMaxDistance = 0.5; // Meters

    public static double kAlignmentSpeed = 0.5; // Percent input to swerves
    public static int[] kCanIDs = {50, 51, 53, 52}; // CAN IDs for the sensors (LeftOuter, LeftInner, RightOuter, RightInner)


    // SIMULATION CONSTANTS
    public static Transform2d[] kCanRangeArrayPositions = { // Same order as kCanIDs
        new Transform2d(new Translation2d(Units.inchesToMeters(14.75), Units.inchesToMeters(0)), Rotation2d.fromDegrees(30)),
        new Transform2d(new Translation2d(Units.inchesToMeters(14.75), Units.inchesToMeters(0)), Rotation2d.fromDegrees(30)),
        new Transform2d(new Translation2d(Units.inchesToMeters(14.75), Units.inchesToMeters(0)), Rotation2d.fromDegrees(-30)),
        new Transform2d(new Translation2d(Units.inchesToMeters(14.75), Units.inchesToMeters(0)), Rotation2d.fromDegrees(-30))
    };
}

