package frc.robot.subsystems.ExampleKraken;

import static frc.robot.subsystems.ExampleKraken.ExampleKrakenConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ExampleKrakenIOSim implements ExampleKrakenIO {
    // Create a simulation model of the motor
    private DCMotorSim motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1), // 1:1 gearbox for the example
            DCMotor.getNEO(1));

    private double appliedVolts = 0.0; // The applied voltage to the motor (can't be read from the motor this is set by us)

    @Override
    public void updateInputs(ExampleKrakenIOInputsAutoLogged inputs) { // called from a periodic method
        motor.setInputVoltage(appliedVolts);
        motor.update(0.02);

        inputs.positionRotations = motor.getAngularPositionRotations();
        inputs.velocityRotationsPerSec = motor.getAngularVelocityRPM() / 60;
        inputs.appliedVoltage = appliedVolts;
        inputs.currentAmps = motor.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0); // Clamp the voltage to -12V to 12V
    }
}