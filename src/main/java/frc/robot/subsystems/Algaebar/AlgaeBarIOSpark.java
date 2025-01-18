package frc.robot.subsystems.Algaebar;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.Algaebar.AlgaeBarConstants.kAlgaeBarCanID;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class AlgaeBarIOSpark implements AlgaeBarIO {
    private final SparkMax motor = new SparkMax(kAlgaeBarCanID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();

    private final SparkMax motor = new SparkMax(kAlgaeBarCanID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();

    public AlgaeBarIOSpark(){
        //could do motor configuration here
    }

    @Override
    public void updateInputs(AlgaeBarIOInputs inputs) {
        ifOk(motor, encoder::getPosition, (value) -> inputs.positionRad = value); //only 
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