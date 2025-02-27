package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorState;

public interface ElevatorIO {
    
    @AutoLog
    public static class ElevatorIOInputs {
        public double leftPositionRotations = 0.0;
        public double leftVelocityRPM = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;

        public double rightPositionRotations = 0.0;
        public double rightVelocityRPM = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;

        public boolean limitSwitch;

        public ElevatorState state = ElevatorState.REST;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    /**
     * Sets the voltage applied to the motor.
     */
    public default void setVoltage(double volts) {
    }
    
    /**
     * Sets the input torque current
     * @param torqueCurrent Requested torque currentx
     */
    public default void setInputTorqueCurrent(double torqueCurrent) {
    }
    /**
     * Set the setpoint for the elevator
     * @param setpoint
     * @implNote This method must clip setpoint to the maximum and minimum values
     */
    public default void updateSetpoint(double setpoint) {
    }

    public default void checkBoundry() {
    }
}