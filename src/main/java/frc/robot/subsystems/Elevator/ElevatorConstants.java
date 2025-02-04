package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    public static final int kLeftMotorCanID = 1;
    public static final int kRightMotorCanID = 2;

    public static final int kLimitSwitchID = 0;

    public static final double kGearRatio = 1.0 / 10.0; // output to input
    public static final double kRotationToInches = kGearRatio * 1.638 * Math.PI; // 1.638 is the diameter of the sprocket in inches
    public static final double kChainToPulleyRatio = 1 / 1; // chain to pulley ratio
    public static final double kStage3HeightProportion = 22.0 / 30.0; // given by cad

    public static final Transform3d kStage1Position = new Transform3d(0.145, 0.0, 0.0, new Rotation3d());
    public static final Transform3d kStage2Position = kStage1Position;//.plus(new Transform3d(0.0,0.0,0.0254, new Rotation3d()));
    public static final Transform3d kStage3Position = kStage2Position;//.plus(new Transform3d(0.0,0.0,0.0254, new Rotation3d()));


    public static final double kL1RotationsRotations = 14;
    public static final double kL2RotationsRotations = 20;
    public static final double kL3RotationsRotations = 31;
    public static final double kL4RotationsRotations = 48;

    public static final double kMinRotations = 0;
    public static final double kMaxRotations = 58;

    public static final double kProportionalGainSpark = 0.03;
    public static final double kIntegralTermSpark = 0;
    public static final double kDerivativeTermSpark = 0;
    public static final double kStaticFrictionTermSpark = 0.0;
    public static final double kGravityTermSpark = 0.0;
    public static final double kTolerance = 0.1;


    public static final double kProportionalGainSim = 1;
    public static final double kIntegralTermSim = 0;
    public static final double kDerivativeTermSim = 1;
    public static final double kStaticFrictionTermSim = 0.0;
    public static final double kGravityTermSim = 0.0;

    public static final int kCurrentLimit = 40;
}