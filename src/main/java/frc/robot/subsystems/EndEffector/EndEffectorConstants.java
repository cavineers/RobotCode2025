package frc.robot.subsystems.EndEffector;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class EndEffectorConstants {
    public static final int kEndEffectorCanID = 30;

    public static final int kLineBreakIR = 50;
    public static final int kLineBreakIR2 = 51;

    public static final int kCoralPresentIR = 8;
    public static final int kCoralLoadedLimit = 9;

    public static final double kEndEffectorShootSpeedL4 = 0.6; // mo powah mo bettah
    public static final double kEndEffectorShootSpeed = 0.6;
    public static final double kEndEffectorIntakeSpeed = 0.20;

    // Motor Configuration
    public static final boolean kInverted = false;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final int kCurrentLimit = 80;

}