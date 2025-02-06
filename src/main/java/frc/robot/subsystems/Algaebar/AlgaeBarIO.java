package frc.robot.subsystems.Algaebar;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeBarIO {
    
    @AutoLog
    public static class AlgaeBarIOInputs {
        public double rotateMotorPositionRad = 0.0;
        public double rotateMotorVelocityRadPerSec = 0.0;
        public double rotateMotorAppliedVolts = 0.0;
        public double rotateMotorCurrentAmps = 0.0;

        public double coralMotorPositionRad = 0.0;
        public double coralMotorVelocityRadPerSec = 0.0;
        public double coralMotorAppliedVolts = 0.0;
        public double coralMotorCurrentAmps = 0.0;
    } 

    public default void updateInputs(AlgaeBarIOInputs inputs){
    }

    public default void setVoltage(double volts){
    }
}