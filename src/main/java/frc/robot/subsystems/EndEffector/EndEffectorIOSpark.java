package frc.robot.subsystems.EndEffector;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.EndEffector.EndEffectorConstants.kEndEffectorCanID;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class EndEffectorIOSpark implements EndEffectorIO {
    private final SparkMax motor = new SparkMax(kEndEffectorCanID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();

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

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}