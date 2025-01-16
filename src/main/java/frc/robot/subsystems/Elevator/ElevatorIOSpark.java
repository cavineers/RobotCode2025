package frc.robot.subsystems.Elevator;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.Elevator.ElevatorConstants.kLeftMotorCanID;
import static frc.robot.subsystems.Elevator.ElevatorConstants.kRightMotorCanID;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

public class ElevatorIOSpark implements ElevatorIO {

    public final SparkMax leftMotor = new SparkMax(kLeftMotorCanID, MotorType.kBrushless);
    public final SparkMax rightMotor = new SparkMax(kRightMotorCanID, MotorType.kBrushless);

    public final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    public final RelativeEncoder rightEncoder = rightMotor.getEncoder();

    public ElevatorIOSpark() {

    }

    public void updateInputs(ElevatorIOInputs inputs) {
        
        updateMotorInputs(inputs, leftMotor, leftEncoder);
        updateMotorInputs(inputs, rightMotor, rightEncoder);
    }

    public void updateMotorInputs (ElevatorIOInputs inputs, SparkMax motor, RelativeEncoder encoder) {

        ifOk(motor, encoder::getPosition, (value) -> inputs.positionRad = value);
        ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        ifOk(
            motor,
                new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    }

    public void setVoltage(double volts) {
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(-volts);
    }
}