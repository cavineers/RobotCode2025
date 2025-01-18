package frc.robot.subsystems.algaebariosim;

import static frc.robot.subsystems.AlgaeBar.AlgaeBarConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class AlgaeIOSim implements AlgaeIO {
    // create a simulation model of the motor
    private DCMotorSim algaebarRotateMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1), // 1:1 gearbox for the example
        DCMotor.getNEO(1));

    private DCMotorSim algaebarCoralMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1),
        DCMotor.getNEO(1));
    
    private double appliedVolts = 0.0; // The applied voltage to the motor (can't be read from the motor this is set by us)

    @Override
    public void updateInputs(AlgaeIOInputs inputs){ // called from a periodic method
        algaebarRotateMotor.setInputVoltage(appliedVolts);
        algaebarRotateMotor.update(0.02);

        inputs.positionRad = algaebarRotateMotor.getAngularPositionRad();
        inputs.velocityRadPerSec = algaebarRotateMotor.getAngularVelocityRadPerSec();
        imputs.appliedVolts = appliedVolts;
        inputs.currentAmps = algaebarRotateMotor.getCurrentDrawAmps();
    } 

    @Override
    public void updateInputs(AlgaeIOInputs inputs){
        algaebarCoralMotor.setInputVoltage(appliedVolts);
        algaebarCoralMotor.update(0.02);

        inputs.positionRad = algaebarCoralMotor.getAngularPositionRad();
        inputs.velocityRadPerSec = algaebarCoralMotor.getAngularVelocityRadPerSec();
        inputs.currentAmps = algaebarCoralMotor.getCurrentDrawAmps();
    }
    
    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0); // clamp the voltage to -12V to 12V
    }  
}