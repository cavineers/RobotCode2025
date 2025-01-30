package frc.robot.subsystems.Funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {

    @AutoLog
    public static class FunnelIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

        public boolean funnelSensor1 = false;
    } 


    public default void updateInputs(FunnelIOInputs inputs) {
    }
    
    public default void setVoltage(double volts) {
    }
}