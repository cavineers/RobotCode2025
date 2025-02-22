package frc.robot.subsystems.Dealgaefier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;

public class DealgaefierIOSim implements DealgaefierIO {
    
    private DCMotorSim deployMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1),
        DCMotor.getNEO(1));

    private DCMotorSim intakeMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1),
        DCMotor.getNEO(1));

    private double deployAppliedVolts = 0.0;
    private double intakeAppliedVolts = 0.0;

    @Override
    public void updateInputs(DealgaefierIOInputs inputs){
        deployMotor.setInputVoltage(deployAppliedVolts);
        intakeMotor.setInputVoltage(intakeAppliedVolts);
        
        deployMotor.update(0.02);
        intakeMotor.update(0.02);

        inputs.deployMotorPositionRad = deployMotor.getAngularPositionRad();
        inputs.deployMotorVelocityRadPerSec = deployMotor.getAngularVelocityRadPerSec();
        inputs.deployMotorAppliedVolts = deployAppliedVolts;
        inputs.deployMotorCurrentAmps = deployMotor.getCurrentDrawAmps();

        inputs.intakeMotorPositionRad = intakeMotor.getAngularPositionRad();
        inputs.intakeMotorVelocityRadPerSec = intakeMotor.getAngularVelocityRadPerSec();
        inputs.intakeMotorAppliedVolts = intakeAppliedVolts;
        inputs.intakeMotorCurrentAmps = intakeMotor.getCurrentDrawAmps();
    }

    public boolean getSensor(DIOSim sensor) {
        return sensor.getValue();
    }

    @Override
    public void setDeployVoltage(double volts) {
        deployAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }
}
