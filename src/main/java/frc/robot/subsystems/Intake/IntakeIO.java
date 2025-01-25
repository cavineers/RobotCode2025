package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;


public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }


    public default void updateInputs(IntakeIOInputs inputs) {
    }


    public default void setVoltage(double volts){
    }

    
}
