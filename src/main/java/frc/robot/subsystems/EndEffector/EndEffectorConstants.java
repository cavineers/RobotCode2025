package frc.robot.subsystems.EndEffector;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class EndEffectorConstants {
    public static final int kEndEffectorCanID = 30;

    public static final int kCoralPresentIR = 1;
    public static final int kCoralLoadedLimit = 2;

    public static final double kEndEffectorShootSpeed = 0.5;
    public static final double kEndEffectorIntakeSpeed = 0.5;

    // Motor Configuration
    public static final boolean kInverted = false;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final int kCurrentLimit = 60;

}