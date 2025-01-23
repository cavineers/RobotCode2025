package frc.robot.subsystems.Elevator;

import static frc.lib.SparkUtil.*; // has a bunch of utility functions for SparkMax

import static frc.robot.subsystems.Elevator.ElevatorConstants.kLeftMotorCanID;
import static frc.robot.subsystems.Elevator.ElevatorConstants.kRightMotorCanID;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorIOSpark implements ElevatorIO {
    
    private final SparkFlex rightMotor = new SparkFlex(kLeftMotorCanID, MotorType.kBrushless);
    private final SparkFlex leftMotor = new SparkFlex(kRightMotorCanID, MotorType.kBrushless);

    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

    PIDController elevPid = new PIDController(ElevatorConstants.kProportionalGain, ElevatorConstants.kIntegralTerm, ElevatorConstants.kDerivativeTerm);

    private double motorSetPoint = 0;

    public ElevatorIOSpark() {

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        updateMotorInputs(inputs, leftMotor, leftEncoder);
        updateMotorInputs(inputs, rightMotor, rightEncoder);
    }

    public void updateMotorInputs(ElevatorIOInputs inputs, SparkFlex motor, RelativeEncoder encoder) {
        ifOk(motor, encoder::getPosition, (value) -> inputs.positionRad = value); 
        ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        ifOk(
            motor,
                new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    }

    public void setVoltage(double volts, SparkFlex motor) {
        motor.setVoltage(volts);
    }

    public double getElevMotorPosition() {
        return rightEncoder.getPosition();
    }

    public void setSetPoint(double setPoint) {
        motorSetPoint = setPoint;
    }

    public void updateSetPoint() {
        elevPid.setSetpoint(motorSetPoint);
        double speed = elevPid.calculate(getElevMotorPosition());
        setVoltage((speed * 12.0), rightMotor);
        setVoltage((speed * 12.0), leftMotor);
    }

}