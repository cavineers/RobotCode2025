package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;

public class ElevatorIOSim implements ElevatorIO {
    
    private DCMotorSim rightMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, kGearRatio),
            DCMotor.getNEO(1));
    
    private DCMotorSim leftMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, kGearRatio),
            DCMotor.getNEO(1));

    private DIOSim limitSwitch = new DIOSim(ElevatorConstants.kLimitSwitchID);

    PIDController elevPid = new PIDController(ElevatorConstants.kProportionalGainSim, ElevatorConstants.kIntegralTermSim, ElevatorConstants.kDerivativeTermSim);

    private double motorSetpoint = 0;
    private double appliedVolts = 0.0; 

    public void updateInputs(ElevatorIOInputs inputs) { 
        updateMotorInputs(inputs, leftMotor);
        updateMotorInputs(inputs, rightMotor);

        double speed = elevPid.calculate(getElevMotorPosition());
        setVoltage(speed * 12.0);
        setVoltage(speed * 12.0);

        inputs.limitSwitch = getLimitSwitch();
    }

    public void updateMotorInputs(ElevatorIOInputs inputs, DCMotorSim motor) {
        motor.setInputVoltage(appliedVolts);
        motor.update(0.02);

        inputs.positionRad = motor.getAngularPositionRad();
        inputs.velocityRadPerSec = motor.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = motor.getCurrentDrawAmps();
    }
    
    public double getElevMotorPosition() {
        return rightMotor.getAngularPositionRad();
    }

    public boolean getLimitSwitch() {
        return limitSwitch.getValue();
    }

    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0); 
    }

    public void setSetpoint(double setpoint) {
        motorSetpoint = setpoint;
    }

    public void updateSetpoint() {
        elevPid.setSetpoint(motorSetpoint);
    }

    public void checkBoundry() {
        if(motorSetpoint > ElevatorConstants.kMaxRotations) {
            motorSetpoint = ElevatorConstants.kMaxRotations;
        } else if(motorSetpoint < ElevatorConstants.kMinRotations) {
            motorSetpoint = ElevatorConstants.kMinRotations;
        }
    }
}