package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;


public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double positionRad = 0.0;
        public double currentAmps = 0.0;
        public double appliedVolts = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVoltsLeft = 0.0;
        public double appliedVoltsRight = 0.0;
        public double positionRadLeft = 0.0;
        public double positionRadRight = 0.0;
        public double currentAmpsLeft = 0.0;
        public double currentAmpsRight = 0.0;
        public double velocityRadPerSecLeft = 0.0;
        public double velocityRadPerSecRight = 0.0;

        public boolean leftSensor = false;
        public boolean rightSensor = true;
   }


    public default void updateInputs(IntakeIOInputs inputs) {
    }


    public default void setVoltage(double volts){
    }

    
}
