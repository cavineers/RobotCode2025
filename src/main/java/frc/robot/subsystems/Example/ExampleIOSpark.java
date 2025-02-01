package frc.robot.subsystems.Example;

import static frc.lib.SparkUtil.*; // has a bunch of utility functions for SparkMax
import static frc.robot.subsystems.Example.ExampleConstants.kExampleCanID;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ExampleIOSpark implements ExampleIO {
    private final SparkMax motor = new SparkMax(kExampleCanID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();

    public ExampleIOSpark() {
        // Could do motor configuration here
        var motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(40);
        tryUntilOk(motor, 5, () -> motor.configure(
            motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    }

    @Override
    public void updateInputs(ExampleIOInputs inputs) {
        ifOk(motor, encoder::getPosition, (value) -> inputs.positionRad = value); // only updates the value if the output is valid
        ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        ifOk(
            motor,
                new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);


        for (int i = 0; i < inputs.recentAmpsHistory.length - 1; i++) {
            inputs.recentAmpsHistory[i] = inputs.recentAmpsHistory[i + 1];
        }
        // Set the last element to currentAmps
        inputs.recentAmpsHistory[inputs.recentAmpsHistory.length - 1] = inputs.currentAmps;
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
