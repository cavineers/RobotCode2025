package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
    
    private DCMotorSim rightMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, kGearRatio),
            DCMotor.getNEO(1));
    
    private DCMotorSim leftMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, kGearRatio),
            DCMotor.getNEO(1));

    PIDController elevPid = new PIDController(ElevatorConstants.kProportionalGain, ElevatorConstants.kIntegralTerm, ElevatorConstants.kDerivativeTerm);

    private double motorSetPoint = 0;

    private double appliedVolts = 0.0; 

    public void updateInputs(ElevatorIOInputs inputs) { 
        updateMotorInputs(inputs, leftMotor);
        updateMotorInputs(inputs, rightMotor);
    }

    public void updateMotorInputs(ElevatorIOInputs inputs, DCMotorSim motor) {
        motor.setInputVoltage(appliedVolts);
        motor.update(0.02);

        inputs.positionRad = motor.getAngularPositionRad();
        inputs.velocityRadPerSec = motor.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = motor.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0); 
    }

    public double getElevMotorPosition() {
        return rightMotor.getAngularPositionRad();
    }

    public void setSetPoint(double setPoint) {
        motorSetPoint = setPoint;
    }

    public void updateSetPoint() {
        elevPid.setSetpoint(motorSetPoint);
        double speed = elevPid.calculate(getElevMotorPosition());
        setVoltage(speed * 12.0);
        setVoltage(speed * 12.0);
    }
}