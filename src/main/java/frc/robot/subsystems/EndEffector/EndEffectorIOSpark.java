package frc.robot.subsystems.EndEffector;

import static frc.lib.SparkUtil.*;
import static frc.robot.subsystems.EndEffector.EndEffectorConstants.kCoralPresentIR;
import static frc.robot.subsystems.EndEffector.EndEffectorConstants.kEndEffectorCanID;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;

public class EndEffectorIOSpark implements EndEffectorIO {
    private final SparkFlex motor = new SparkFlex(kEndEffectorCanID, MotorType.kBrushless);

    private final RelativeEncoder encoder = motor.getEncoder();

    private final DigitalInput coralPresentIR = new DigitalInput(EndEffectorConstants.kCoralPresentIR);
    private final DigitalInput coralLoadedLimit = new DigitalInput(EndEffectorConstants.kCoralLoadedLimit);

    private SparkFlexConfig config;

    public EndEffectorIOSpark() {
        config = new SparkFlexConfig();
        config
            .inverted(EndEffectorConstants.kInverted)
            .idleMode(EndEffectorConstants.kIdleMode)
            .smartCurrentLimit(EndEffectorConstants.kCurrentLimit)
            .voltageCompensation(12);
        config.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);        
            
        tryUntilOk(
            motor,
            5,
            () -> motor.configure(config, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        ifOk(motor, encoder::getPosition, (value) -> inputs.positionRad = value); 
        ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        ifOk(
            motor,
                new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
        inputs.coralLoadedLimit = this.getBumpStop();
        inputs.coralPresentIR = this.getIR();
    }

    public boolean getBumpStop(){
        return !this.coralLoadedLimit.get();
    }

    public boolean getIR(){
        return this.coralPresentIR.get();
    }

    public boolean getSensor(DigitalInput sensor) {
        return !sensor.get();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void intake() {
        if(getSensor(coralLoadedLimit) == false) {
            motor.setVoltage(EndEffectorConstants.kEndEffectorIntakeSpeed * 12.0);
        } else {
            motor.setVoltage(0.0);
        }
    }

    @Override
    public void shoot() {
        if(getSensor(coralLoadedLimit) == false) {
            motor.setVoltage(EndEffectorConstants.kEndEffectorShootSpeed * 12.0);
        } else {
            motor.setVoltage(0.0);
        }
    }
}