package frc.robot.subsystems.Intake;

import static frc.robot.subsystems.Intake.IntakeConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpiibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    // create a simulation model of the motor
    private DCMotorSim motor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.get(1), 0.004, 1), // 1:1 gearbox for the ex
        DCMotor.getNEO(1));

    private double appliedVolts = 0.0; // the applied voltagge to the motor (can't be read from the motor)

    @Override
    public void updateInputs(IntaekIOInputs inputs) {//called from a peridic method
        motor.setInputVoltage(appliedVolts);
        motor.update(0.02);

        inputs.positionRad = motor.getAngularPositonRad();
        inputs.velocityRadPerSec = motor.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmpsp = motor.getCurrentDrawAmps;
    }

    @Override
    public void setVoltage(double volts){
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0); //clamp voltage to 12V -12V
    }
  
}