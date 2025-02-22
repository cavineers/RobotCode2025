package frc.robot.subsystems.Dealgaefier;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.Dealgaefier.DealgaefierConstants.kDeployCanID;
import static frc.robot.subsystems.Dealgaefier.DealgaefierConstants.kIntakeCanID;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public class DealgaefierIOSpark implements DealgaefierIO {
    final SparkMax deployMotor = new SparkMax(kDeployCanID, MotorType.kBrushless);
    final RelativeEncoder deployEncoder = deployMotor.getEncoder();

    final SparkMax intakeMotor = new SparkMax(kIntakeCanID, MotorType.kBrushless);
    final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    public DealgaefierIOSpark(){

    }

    @Override
    public void updateInputs(DealgaefierIOInputs inputs) {
        ifOk(deployMotor, deployEncoder::getPosition, (value) -> inputs.deployMotorPositionRad = value);
        ifOk(deployMotor, deployEncoder::getVelocity, (value) -> inputs.deployMotorVelocityRadPerSec = value);
        ifOk(
            deployMotor,
            new DoubleSupplier[] {deployMotor::getAppliedOutput, deployMotor::getBusVoltage},
            (values -> inputs.deployMotorAppliedVolts = values[0] * values[1]));
        ifOk(deployMotor, deployMotor::getOutputCurrent, (value) -> inputs.deployMotorCurrentAmps = value);
        
        ifOk(intakeMotor, intakeEncoder::getPosition, (value) -> inputs.intakeMotorPositionRad = value);
        ifOk(intakeMotor, intakeEncoder::getVelocity, (value) -> inputs.intakeMotorVelocityRadPerSec = value);
        ifOk(
            intakeMotor,
            new DoubleSupplier[] {intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage},
            (values -> inputs.intakeMotorAppliedVolts = values[0] * values[1]));
        ifOk(intakeMotor, intakeMotor::getOutputCurrent, (value) -> inputs.intakeMotorCurrentAmps = value);
    }

    public boolean getSensor(DigitalInput sensor) {
        return sensor.get();
    }
    
    @Override
    public void setDeployVoltage(double volts) {
        deployMotor.setVoltage(volts);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }
}
