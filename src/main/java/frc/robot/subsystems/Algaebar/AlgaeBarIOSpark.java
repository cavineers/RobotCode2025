package frc.robot.subsystems.Algaebar;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.Algaebar.AlgaeBarConstants.kAlgaeBarRotateCanID;
import static frc.robot.subsystems.Algaebar.AlgaeBarConstants.kAlgaeBarCoralCanID;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class AlgaeBarIOSpark implements AlgaeBarIO {
    private final SparkMax rotateMotor = new SparkMax(kAlgaeBarRotateCanID, MotorType.kBrushless); //change CanID names as appropriate
    private final RelativeEncoder rotateEncoder = rotateMotor.getEncoder();

    private final SparkMax coralMotor = new SparkMax(kAlgaeBarCoralCanID, MotorType.kBrushless);
    private final RelativeEncoder coralEncoder = coralMotor.getEncoder();

    public DigitalInput firstSensor = new DigitalInput(AlgaeBarConstants.algaeBarSensorOne); //change sensor names as appropriate
    public DigitalInput secondSensor = new DigitalInput(AlgaeBarConstants.algaeBarSensorTwo);

    public AlgaeBarIOSpark(){
        //could do motor configuration here
    }

    @Override
    public void updateInputs(AlgaeBarIOInputs inputs) {
        ifOk(rotateMotor, rotateEncoder::getPosition, (value) -> inputs.positionRad = value); //only 
        ifOk(rotateMotor, rotateEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        ifOk(
            rotateMotor,
                new DoubleSupplier[] {rotateMotor::getAppliedOutput, rotateMotor::getBusVoltage},
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(rotateMotor, rotateMotor::getOutputCurrent, (value) -> inputs.currentAmps = value);   
        
        ifOk(coralMotor, coralEncoder::getPosition, (value) -> inputs.positionRad = value); 
        ifOk(coralMotor, coralEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        ifOk(
            coralMotor,
                new DoubleSupplier[] {coralMotor::getAppliedOutput, coralMotor::getBusVoltage},
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(coralMotor, coralMotor::getOutputCurrent, (value) -> inputs.currentAmps = value);   

        if (isSensorHit(firstSensor) || isSensorHit(secondSensor)) {
            inputs.velocityRadPerSec = 0;
        } else {
            ifOk(rotateMotor, rotateEncoder::getPosition, value -> inputs.positionRad = value);
            ifOk(coralMotor, coralEncoder::getPosition, value -> inputs.positionRad = value);
    }
    } 

    @Override
    public void setVoltage(double volts) {
            rotateMotor.setVoltage(volts);

            coralMotor.setVoltage(volts);
        }

        private boolean isSensorHit(DigitalInput sensor) {
            return sensor.get();
        }
 }