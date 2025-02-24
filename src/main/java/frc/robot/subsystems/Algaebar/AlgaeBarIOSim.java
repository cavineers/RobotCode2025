package frc.robot.subsystems.Algaebar;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class AlgaeBarIOSim implements AlgaeBarIO {

    private DCMotorSim algaeBarPivotMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1), 
        DCMotor.getNEO(1));

    private DCMotorSim algaeBarMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1),
        DCMotor.getNEO(1));
        
    private double algaeBarPivotMotorAppliedVolts = 0.0; 
    private double algaeBarMotorAppliedVolts = 0.0;

    @Override
    public void updateInputs(AlgaeBarIOInputs inputs) { 
        inputs.pivotMotorPositionRad = algaeBarPivotMotor.getAngularPositionRad();
        inputs.pivotMotorVelocityRadPerSec = algaeBarPivotMotor.getAngularVelocityRadPerSec();
        inputs.pivotMotorAppliedVolts = algaeBarPivotMotorAppliedVolts;
        inputs.pivotMotorCurrentAmps = algaeBarPivotMotor.getCurrentDrawAmps();

        inputs.algaeBarMotorPositionRad = algaeBarMotor.getAngularPositionRad();
        inputs.algaeBarMotorVelocityRadPerSec = algaeBarMotor.getAngularVelocityRadPerSec();
        inputs.algaeBarMotorAppliedVolts = algaeBarMotorAppliedVolts;
        inputs.algaeBarMotorCurrentAmps = algaeBarMotor.getCurrentDrawAmps();
    } 
    
    @Override
    public void setPivotVoltage(double volts) {
        algaeBarPivotMotorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0); 
    }  
    @Override
    public void setAlgaeBarVoltage(double volts) {
        algaeBarMotorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0); 
    }
}