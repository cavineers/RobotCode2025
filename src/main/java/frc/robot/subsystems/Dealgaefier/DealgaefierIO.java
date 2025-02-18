package frc.robot.subsystems.Dealgaefier;

import org.littletonrobotics.junction.AutoLog;

public interface DealgaefierIO {
    @AutoLog
    public static class DealgaefierIOInputs{
        public double pivotMotorPositionRad = 0.0;
        public double pivotMotorVelocityRadPerSec = 0.0;
        public double pivotMotorAppliedVolts = 0.0;
        public double pivotMotorCurrentAmps = 0.0;

        public double spinMotorPositionRad = 0.0;
        public double spinMotorVelocityRadPerSec = 0.0;
        public double spinMotorAppliedVolts = 0.0;
        public double spinMotorCurrentAmps = 0.0;

        public boolean dealgaefierLimit = false;
    }
    
    default void updateInputs(DealgaefierIOInputs inputs) {
        }

    public default void setVoltage(double volts) {
        }

    public default void pivot(){
    }
}
