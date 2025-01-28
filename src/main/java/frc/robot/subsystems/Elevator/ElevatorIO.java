package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public boolean limitSwitch;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void setVoltage(double volts) {
    }

    public default void setSetpoint(double setpoint) {
    }

    public default void updateSetpoint() {
    }

    public default void checkBoundry() {
    }
}