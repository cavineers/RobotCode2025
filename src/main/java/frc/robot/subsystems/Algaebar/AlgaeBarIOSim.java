package frc.robot.subsystems.Algaebar;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class AlgaeBarIOSim implements AlgaeBarIO {

    private DCMotorSim algaebarRotateMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1), 
        DCMotor.getNEO(1));

    private DCMotorSim algaebarCoralMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1),
        DCMotor.getNEO(1));
        
    private double appliedVolts = 0.0; 

    @Override
    public void updateInputs(AlgaeBarIOInputs inputs) { 
        inputs.rotateMotorPositionRad = algaebarRotateMotor.getAngularPositionRad();
        inputs.rotateMotorVelocityRadPerSec = algaebarRotateMotor.getAngularVelocityRadPerSec();
        inputs.rotateMotorAppliedVolts = appliedVolts;
        inputs.rotateMotorCurrentAmps = algaebarRotateMotor.getCurrentDrawAmps();

        inputs.coralMotorPositionRad = algaebarCoralMotor.getAngularPositionRad();
        inputs.coralMotorVelocityRadPerSec = algaebarCoralMotor.getAngularVelocityRadPerSec();
        inputs.coralMotorAppliedVolts = appliedVolts;
        inputs.coralMotorCurrentAmps = algaebarCoralMotor.getCurrentDrawAmps();
    } 
    
    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0); 
    }  
}