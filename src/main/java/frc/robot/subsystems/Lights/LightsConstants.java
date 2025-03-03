package frc.robot.subsystems.Lights;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class LightsConstants {
    public enum LightsState {
        ElevatorMovement,
        ShooterRunning,
        IntakeStandby,
        IntakeAvailable,
        IntakeSuccess,
        Default
    } 
    public static final double kElevatorVelocityThreshold = 25.0; // RPM to be considered moving
    public static final int kPWMPort = 5;

    public static final Distance kLedSpacing = Units.Meters.of(1 / 120.0);

    public static final int kFunnelCount = 21; // number of LEDs on a single side
    public static final int kTopCount = 15; // number of LEDs on a single side
    public static final int kElevatorCount = 20; // number of LEDs on a single side
    public static final int kRightSideStart = kFunnelCount + kTopCount + kElevatorCount; // start of the right side of the LED strip
}