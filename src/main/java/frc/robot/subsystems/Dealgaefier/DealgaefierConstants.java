package frc.robot.subsystems.Dealgaefier;

public class DealgaefierConstants {
    public static final boolean kTuningMode = true;

    public static final int kDeployCanID = 53;
    public static final int kIntakeCanID = 52;

    public static final double kRestAbsoluteRotations = 0.2;
    public static final double kDeployedAbsoluteRotations = 0.5;

    public static final int kDeployAbsEncoder = 4 ;

    public static final double kProportionalGainSpark = 10;
    public static final double kIntegralTermSpark = 0.0;
    public static final double kDerivativeTermSpark = 0.0;
    public static final double kGravityTermSpark = 0.0;

    public static final double kProportionalTermSim = 0.1;
    public static final double kDerivativeTermSim = 0.0;

    public static final double kDeployGearRatio = 1;
    public static final double kIntakeGearRatio = 1;

    public static final double kIntakeSpeed = 0.2;

    public static final boolean kInverted = false;

    public static final int kDeployCurrentLimit = 40;
    public static final int kIntakeCurrentLimit = 20;
}
