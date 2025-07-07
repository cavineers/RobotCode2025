package frc.robot.subsystems.Dealgaefier;

import static frc.robot.subsystems.Dealgaefier.DealgaefierConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class DealgaefierIOSim implements DealgaefierIO {
    
    private DCMotorSim deployMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1 / DealgaefierConstants.kDeployGearRatio),
        DCMotor.getNEO(1));

    private DCMotorSim intakeMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1), // not applying any gear ratio here bc it doesn't matter
        DCMotor.getNEO(1));

    private LoggedNetworkNumber tuningP = new LoggedNetworkNumber("Tuning/Dealgaefier/P", kProportionalTermSim);
    private LoggedNetworkNumber tuningD = new LoggedNetworkNumber("Tuning/Dealgaefier/I", kDerivativeTermSim);
    
    private PIDController deployController = new PIDController(tuningP.get(), 0.0, tuningD.get());

    private double deployAppliedVolts = 0.0;
    private double intakeAppliedVolts = 0.0;

    @AutoLogOutput(key="Dealgaefier/Setpoint")
    private double deploySetpoint = 0.0;

    @Override
    public void updateInputs(DealgaefierIOInputs inputs){

        if (tuningP.get() != deployController.getP() || tuningD.get() != deployController.getD()) {
            deployController.setPID(tuningP.get(), 0.0, tuningD.get());
        }

        double absoluteRotations = this.deployMotor.getAngularPositionRotations() * kDeployGearRatio;

        deployMotor.setInputVoltage(deployController.calculate(absoluteRotations));
        intakeMotor.setInputVoltage(intakeAppliedVolts);
        
        deployMotor.update(0.02);
        intakeMotor.update(0.02);

        inputs.deployMotorPositionRotations = deployMotor.getAngularPositionRad();
        inputs.deployMotorVelocityRadPerSec = deployMotor.getAngularVelocityRadPerSec();
        inputs.deployMotorAppliedVolts = deployAppliedVolts;
        inputs.deployMotorCurrentAmps = deployMotor.getCurrentDrawAmps();

        inputs.intakeMotorPositionRotations = intakeMotor.getAngularPositionRad();
        inputs.intakeMotorVelocityRadPerSec = intakeMotor.getAngularVelocityRadPerSec();
        inputs.intakeMotorAppliedVolts = intakeAppliedVolts;
        inputs.intakeMotorCurrentAmps = intakeMotor.getCurrentDrawAmps();
    }

    @Override
    public void setDeployVoltage(double volts) {
        deployAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override 
    public void updateSetpoint(double rotations){
        this.deploySetpoint = rotations;
        deployController.setSetpoint(this.deploySetpoint);
    }
}
