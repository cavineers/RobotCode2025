package frc.robot.subsystems.EndEffector;

import static frc.robot.subsystems.EndEffector.EndEffectorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;

public class EndEffectorIOSim implements EndEffectorIO {

    private DCMotorSim motor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1), 
            DCMotor.getNEO(1));

    private static DIOSim coralPresentIR = new DIOSim(EndEffectorConstants.kCoralPresentIR);
    private static DIOSim coralLoadedLimit = new DIOSim(EndEffectorConstants.kCoralLoadedLimit);

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) { 
        coralPresentIR.setValue(true);
        coralLoadedLimit.setValue(false);
        motor.setInputVoltage(appliedVolts);
        motor.update(0.02);

        inputs.positionRad = motor.getAngularPositionRad();
        inputs.velocityRadPerSec = motor.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = motor.getCurrentDrawAmps();

        inputs.coralPresentIR = false;
        inputs.coralLoadedLimit = true;
        inputs.isFunneling = true;
    }

    public boolean getSensor(DIOSim sensor) {
        return sensor.getValue();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0); 
    }

    public void intake() {
        if(getSensor(coralLoadedLimit) == false) {
            setVoltage(EndEffectorConstants.kEndEffectorIntakeSpeed * 12.0);
        }
    }

    public void shoot() {
        if(getSensor(coralLoadedLimit) == false) {
            setVoltage(EndEffectorConstants.kEndEffectorShootSpeed * 12.0);
        }
    }
}
