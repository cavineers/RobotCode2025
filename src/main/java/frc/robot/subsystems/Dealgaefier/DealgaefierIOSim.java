package frc.robot.subsystems.Dealgaefier;

import static frc.robot.subsystems.Dealgaefier.DealgaefierConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;

public class DealgaefierIOSim implements DealgaefierIO {
    
    private DCMotorSim deployMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1 / DealgaefierConstants.kDeployGearRatio),
        DCMotor.getNEO(1));

    private DCMotorSim intakeMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1), // not applying any gear ratio here bc it doesn't matter
        DCMotor.getNEO(1));


    private double deployAppliedVolts = 0.0;
    private double intakeAppliedVolts = 0.0;

    @AutoLogOutput(key="Dealgaefier/Setpoint")

    @Override
    public void updateInputs(DealgaefierIOInputs inputs){

    
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

        // Continuous from 0 to 1
        inputs.absolutePosition = deployMotor.getAngularPositionRotations() * kDeployGearRatio;
    }

    @Override
    public void setDeployVoltage(double volts) {
        deployAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }
}
