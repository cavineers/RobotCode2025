package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;


public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorIOInputs {
        public double positionRad = 0.0;
        public double currentAmps = 0.0;
        public double appliedVolts = 0.0;
        public double velocityRadPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double rightAppliedVolts = 0.0;
        public double leftPositionRad = 0.0;
        public double rightPositionRad = 0.0;
        public double leftCurrentAmps = 0.0;
        public double rightCurrentAmps = 0.0;
        public double leftVelocityRadPerSec = 0.0;
        public double rightVelocityRadPerSec = 0.0;

        public boolean leftSensor = false;
        public boolean rightSensor = true;
   }


    public default void updateInputs(EndEffectorIOInputs inputs) {
    }


    public default void setVoltage(double volts){
    }

    
}
