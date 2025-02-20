package frc.robot.subsystems.EndEffector;

import static frc.lib.SparkUtil.*;
import static frc.robot.subsystems.EndEffector.EndEffectorConstants.kCoralPresentIR;
import static frc.robot.subsystems.EndEffector.EndEffectorConstants.kEndEffectorCanID;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;

public class EndEffectorIOSpark implements EndEffectorIO {
    private final SparkMax motor = new SparkMax(kEndEffectorCanID, MotorType.kBrushless);

    private final RelativeEncoder encoder = motor.getEncoder();

    private final DigitalInput coralPresentIR = new DigitalInput(EndEffectorConstants.kCoralPresentIR);
    private final DigitalInput coralLoadedLimit = new DigitalInput(EndEffectorConstants.kCoralLoadedLimit);


    public EndEffectorIOSpark() {

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
    }

    public boolean getSensor(DigitalInput sensor) {
        return sensor.get();
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