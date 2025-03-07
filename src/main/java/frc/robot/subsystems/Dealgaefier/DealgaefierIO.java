package frc.robot.subsystems.Dealgaefier;

import org.littletonrobotics.junction.AutoLog;

public interface DealgaefierIO {
    @AutoLog
    public static class DealgaefierIOInputs{
        public double deployMotorPositionRotations = 0.0;
        public double deployMotorVelocityRadPerSec = 0.0;
        public double deployMotorAppliedVolts = 0.0;
        public double deployMotorCurrentAmps = 0.0;

        public double intakeMotorPositionRotations = 0.0;
        public double intakeMotorVelocityRadPerSec = 0.0;
        public double intakeMotorAppliedVolts = 0.0;
        public double intakeMotorCurrentAmps = 0.0;
    }
    
    default void updateInputs(DealgaefierIOInputs inputs) {
    }

    public default void setDeployVoltage(double volts) {
    }

    public default void setIntakeVoltage(double volts) {
    }

    public default void initializeDutyEncoder() {
    }

    public default void updateSetpoint(double setpoint) {
    }

    public default void deploy() {
    }

    public default void retract() {
    }

    public default void shoot() {
        
    }
}
