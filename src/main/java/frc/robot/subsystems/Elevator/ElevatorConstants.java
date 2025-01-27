package frc.robot.subsystems.Elevator;

public class ElevatorConstants {
    public static final int kLeftMotorCanID = 1;
    public static final int kRightMotorCanID = 2;
    public static final int kGearRatio = 1;

    public static final double kL1RotationsRad = 100;
    public static final double kL2RotationsRad = 200;
    public static final double kL3RotationsRad = 300;
    public static final double kL4RotationsRad = 400;

    public static final double kMinRotations = 0;
    public static final double kMaxRotations = 50;

    public static final double kProportionalGainSpark = 0.03;
    public static final double kIntegralTermSpark = 0;
    public static final double kDerivativeTermSpark = 0;

    public static final double kProportionalGainSim = 0.004;
    public static final double kIntegralTermSim = 0;
    public static final double kDerivativeTermSim = 0;
}