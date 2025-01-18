package frc.robot.subsystem.algaebario;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeBarIO {
    
    @AutoLog
    public static class AlgaeBarIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    } 

    /**Update the set of loggable inputs. */
    public default void updateInputs(AlgaeBarIOInputs inputs){
    }

    /** Run open loop at the specified voltage.*/
    public default void setVoltage(double volts){
    }
}