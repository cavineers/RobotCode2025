package frc.robot.subsystems.Dealgaefier;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.Dealgaefier.DealgaefierConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class DealgaefierIOSpark implements DealgaefierIO {
    final SparkMax deployMotor = new SparkMax(kDeployCanID, MotorType.kBrushless);
    final RelativeEncoder deployEncoder = deployMotor.getEncoder();

    final SparkMax intakeMotor = new SparkMax(kIntakeCanID, MotorType.kBrushless);
    final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    public DutyCycleEncoder deployAbsEncoder = new DutyCycleEncoder(DealgaefierConstants.kDeployAbsEncoder);

    @AutoLogOutput(key="Dealgaefier/Setpoint")

    private SparkMaxConfig deployConfig;

    public boolean absEncoderInitialized = false;
    
    public DealgaefierIOSpark(){    
        deployConfig = new SparkMaxConfig();
        deployConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DealgaefierConstants.kCurrentLimit)    
            .voltageCompensation(12); 
     
        tryUntilOk(
            deployMotor,
            5,
            () -> deployMotor.configure(deployConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));

        var intakeConfig = new SparkMaxConfig().apply(deployConfig);
        
        intakeConfig.inverted(true);
        tryUntilOk(
            intakeMotor,
            5,
            () -> intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(DealgaefierIOInputs inputs) {

        ifOk(deployMotor, deployEncoder::getPosition, (value) -> inputs.deployMotorPositionRotations = value);
        ifOk(deployMotor, deployEncoder::getVelocity, (value) -> inputs.deployMotorVelocityRadPerSec = value);
        ifOk(
            deployMotor,
            new DoubleSupplier[] {deployMotor::getAppliedOutput, deployMotor::getBusVoltage},
            (values -> inputs.deployMotorAppliedVolts = values[0] * values[1]));
        ifOk(deployMotor, deployMotor::getOutputCurrent, (value) -> inputs.deployMotorCurrentAmps = value);
        
        ifOk(intakeMotor, intakeEncoder::getPosition, (value) -> inputs.intakeMotorPositionRotations = value);
        ifOk(intakeMotor, intakeEncoder::getVelocity, (value) -> inputs.intakeMotorVelocityRadPerSec = value);
        ifOk(
            intakeMotor,
            new DoubleSupplier[] {intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage},
            (values -> inputs.intakeMotorAppliedVolts = values[0] * values[1]));
        ifOk(intakeMotor, intakeMotor::getOutputCurrent, (value) -> inputs.intakeMotorCurrentAmps = value);

        inputs.absolutePosition = getAbsEncoder();
        if(absEncoderInitialized == false) {
            initializeDutyEncoder();
        }
    }

    public void initializeDutyEncoder(){
        absEncoderInitialized = true;
    }

    public boolean getSensor(DigitalInput sensor) {
        return sensor.get();
    }

    public double getAbsEncoder() {
        return this.deployAbsEncoder.get() + kAbsEncoderOffset;
    }

    public double getDeployPositionRotations() {
        return deployEncoder.getPosition();
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
