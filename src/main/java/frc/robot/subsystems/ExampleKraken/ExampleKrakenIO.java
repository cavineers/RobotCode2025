package frc.robot.subsystems.ExampleKraken;

import org.littletonrobotics.junction.AutoLog;

public interface ExampleKrakenIO {
    @AutoLog
    public static class ExampleKrakenIOInputs {
        public double velocityRotationsPerSec = 0.0;
        public double appliedVoltage = 0.0;
        public double currentAmps = 0.0;
        public double positionRotations = 0.0;
    }

    /** Update the set of loggable inputs. */
    public default void updateInputs(ExampleKrakenIOInputs inputs) {
    }

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {
    }
}