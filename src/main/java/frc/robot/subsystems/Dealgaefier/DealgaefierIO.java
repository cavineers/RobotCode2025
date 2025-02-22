package frc.robot.subsystems.Dealgaefier;

import org.littletonrobotics.junction.AutoLog;

public interface DealgaefierIO {
    @AutoLog
    public static class DealgaefierIOInputs{
        public double deployMotorPositionRad = 0.0;
        public double deployMotorVelocityRadPerSec = 0.0;
        public double deployMotorAppliedVolts = 0.0;
        public double deployMotorCurrentAmps = 0.0;

        public double intakeMotorPositionRad = 0.0;
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
}
