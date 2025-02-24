package frc.robot.subsystems.Algaebar;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeBarIO {
    
    @AutoLog
    public static class AlgaeBarIOInputs {
        public double pivotMotorPositionRad = 0.0;
        public double pivotMotorVelocityRadPerSec = 0.0;
        public double pivotMotorAppliedVolts = 0.0;
        public double pivotMotorCurrentAmps = 0.0;

        public double algaeBarMotorPositionRad = 0.0;
        public double algaeBarMotorVelocityRadPerSec = 0.0;
        public double algaeBarMotorAppliedVolts = 0.0;
        public double algaeBarMotorCurrentAmps = 0.0;
    } 

    public default void updateInputs(AlgaeBarIOInputs inputs){
    }

    public default void setPivotVoltage(double volts){
    }
    public default void setAlgaeBarVoltage(double volts){
    }

    public default void initializeDutyEncoder(){
    }
}