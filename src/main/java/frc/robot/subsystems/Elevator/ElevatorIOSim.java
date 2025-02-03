package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;

public class ElevatorIOSim implements ElevatorIO {
    
    private DCMotorSim rightMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(2), 0.004, kGearRatio),
            DCMotor.getNeoVortex(2));
    
    // private DCMotorSim leftMotor = new DCMotorSim(
    //         LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.004, kGearRatio),
    //         DCMotor.getNeoVortex(1));

    private DIOSim limitSwitch = new DIOSim(ElevatorConstants.kLimitSwitchID);

    PIDController elevPid = new PIDController(ElevatorConstants.kProportionalGainSim, ElevatorConstants.kIntegralTermSim, ElevatorConstants.kDerivativeTermSim);

    @AutoLogOutput(key = "Elevator/Setpoint")
    private double motorSetpoint = 0;
    private double appliedVolts = 0.0; 

    public void updateInputs(ElevatorIOInputs inputs) { 
        // Apply voltages updates to motors
        appliedVolts = elevPid.calculate(rightMotor.getAngularPositionRotations()) + kGravityTermSim;
        rightMotor.setInputVoltage(appliedVolts);
        rightMotor.update(0.02);

        inputs.leftPositionRotations = rightMotor.getAngularPositionRotations();
        inputs.leftVelocityRPM = rightMotor.getAngularVelocityRPM();
        inputs.leftAppliedVolts = rightMotor.getInputVoltage();
        inputs.leftCurrentAmps = rightMotor.getCurrentDrawAmps();

        inputs.rightPositionRotations = rightMotor.getAngularPositionRotations();
        inputs.rightVelocityRPM = rightMotor.getAngularVelocityRPM();
        inputs.rightAppliedVolts = rightMotor.getInputVoltage();
        inputs.rightCurrentAmps = rightMotor.getCurrentDrawAmps();

        inputs.limitSwitch = getLimitSwitch();
    }


    public boolean getLimitSwitch() {
        return limitSwitch.getValue();
    }

    @Deprecated
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0); 
    }

    public void updateSetpoint(double motorSetpoint) {
        this.motorSetpoint = clipSetpoint(motorSetpoint);
        elevPid.setSetpoint(motorSetpoint);
    }

    public double clipSetpoint(double setpoint) {
        if(motorSetpoint > ElevatorConstants.kMaxRotations) {
            return ElevatorConstants.kMaxRotations;
        } else if(motorSetpoint < ElevatorConstants.kMinRotations) {
            return ElevatorConstants.kMinRotations;
        }
        return setpoint;
    }
}