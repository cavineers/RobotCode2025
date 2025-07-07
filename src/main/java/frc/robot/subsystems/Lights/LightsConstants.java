package frc.robot.subsystems.Lights;

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

}