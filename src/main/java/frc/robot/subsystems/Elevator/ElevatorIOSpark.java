package frc.robot.subsystems.Elevator;

import static frc.lib.SparkUtil.*; // has a bunch of utility functions for SparkMax

import static frc.robot.subsystems.Elevator.ElevatorConstants.kLeftMotorCanID;
import static frc.robot.subsystems.Elevator.ElevatorConstants.kRightMotorCanID;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ElevatorIOSpark implements ElevatorIO {
    private final SparkMax rightMotor = new SparkMax(kLeftMotorCanID, MotorType.kBrushless);
    private final SparkMax leftMotor = new SparkMax(kRightMotorCanID, MotorType.kBrushless);
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

    public ElevatorIOSpark() {

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        updateMotorInputs(inputs, leftMotor, leftEncoder);
        updateMotorInputs(inputs, rightMotor, rightEncoder);
    }

    public void updateMotorInputs(ElevatorIOInputs inputs, SparkMax motor, RelativeEncoder encoder) {
        ifOk(motor, encoder::getPosition, (value) -> inputs.positionRad = value); 
        ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        ifOk(
            motor,
                new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    }

    public void setVoltage(double volts, SparkMax motor) {
        motor.setVoltage(volts);
    }
}