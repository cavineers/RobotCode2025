package frc.robot.subsystems.EndEffector;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;


public class EndEffectorIOSim implements EndEffectorIO {
   
    private DCMotorSim motor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1), 
        DCMotor.getNEO(1));

    private double appliedVolts = 0.0; 

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        motor.setInputVoltage(appliedVolts);
        motor.update(0.02);

        inputs.positionRad = motor.getAngularPositionRad();
        inputs.velocityRadPerSec = motor.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = motor.getCurrentDrawAmps();
    }


    @Override
    public void setVoltage(double volts){ 
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

}
