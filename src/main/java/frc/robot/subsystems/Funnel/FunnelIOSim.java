package frc.robot.subsystems.Funnel;
import static frc.robot.subsystems.Funnel.FunnelConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;


public class FunnelIOSim implements FunnelIO {

    private DCMotorSim funnelMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1),
        DCMotor.getNEO(1));

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(FunnelIOInputs inputs) {
        funnelMotor.setInputVoltage(appliedVolts);
        funnelMotor.update(0.02);

        inputs.positionRad = funnelMotor.getAngularPositionRad();
        inputs.velocityRadPerSec = funnelMotor.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = funnelMotor.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }
}