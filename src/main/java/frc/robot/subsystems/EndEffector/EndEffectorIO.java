package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

        public boolean coralPresentIR = false;
        public boolean coralLoadedLimit = false;
    }

    public default void updateInputs(EndEffectorIOInputs inputs) {
    }

    public default void setVoltage(double volts) {
    }

    public default void shoot() {
    }

    public default void intake() {}
}