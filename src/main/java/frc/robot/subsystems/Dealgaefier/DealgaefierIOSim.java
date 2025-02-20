package frc.robot.subsystems.Dealgaefier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import static frc.robot.subsystems.Dealgaefier.DealgaefierConstants.kDealgaefierLimit;

public class DealgaefierIOSim implements DealgaefierIO {
    
    private DCMotorSim motor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1),
        DCMotor.getNEO(1));

    private static DIOSim dealgaefierLimit = new DIOSim(DealgaefierConstants.kDealgaefierLimit);

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(DealgaefierIOInputs inputs){
        motor.setInputVoltage(appliedVolts);
        motor.update(0.02);

        inputs.pivotMotorPositionRad = motor.getAngularPositionRad();
        inputs.pivotMotorVelocityRadPerSec = motor.getAngularVelocityRadPerSec();
        inputs.pivotMotorAppliedVolts = appliedVolts;
        inputs.pivotMotorCurrentAmps = motor.getCurrentDrawAmps();

        inputs.spinMotorPositionRad = motor.getAngularPositionRad();
        inputs.spinMotorVelocityRadPerSec = motor.getAngularVelocityRadPerSec();
        inputs.spinMotorAppliedVolts = appliedVolts;
        inputs.spinMotorCurrentAmps = motor.getCurrentDrawAmps();

        inputs.dealgaefierLimit = getSensor(dealgaefierLimit);
    }

    public boolean getSensor(DIOSim sensor) {
        return sensor.getValue();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

        public void pivot() {
        if(getSensor(dealgaefierLimit) == false) {
            setVoltage(DealgaefierConstants.kDealgaefierPivotSpeed);
        }
    }
}
