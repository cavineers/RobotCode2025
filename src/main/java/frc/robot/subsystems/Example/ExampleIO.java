package frc.robot.subsystems.Example;

import org.littletonrobotics.junction.AutoLog;

public interface ExampleIO {
    @AutoLog
    public static class ExampleIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double[] recentAmpsHistory = new double[20]; // 10 shifting entries
    }

    /** Update the set of loggable inputs. */
    public default void updateInputs(ExampleIOInputs inputs) {
    }

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {
    }
}