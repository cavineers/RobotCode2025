package frc.robot.subsystems.Algaebar;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.Algaebar.AlgaeBarConstants.kAlgaeBarRotateCanID;
import static frc.robot.subsystems.Algaebar.AlgaeBarConstants.kAlgaeBarCoralCanID;


import java.util.function.DoubleSupplier;

//import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeBarIOSpark implements AlgaeBarIO {
    private final SparkMax rotateMotor = new SparkMax(kAlgaeBarRotateCanID, MotorType.kBrushless); 
    private final RelativeEncoder rotateEncoder = rotateMotor.getEncoder();

    private final SparkMax coralMotor = new SparkMax(kAlgaeBarCoralCanID, MotorType.kBrushless);
    private final RelativeEncoder coralEncoder = coralMotor.getEncoder();

    public DigitalInput firstSensor = new DigitalInput(AlgaeBarConstants.algaeBarSensorOne); 
    public DigitalInput secondSensor = new DigitalInput(AlgaeBarConstants.algaeBarSensorTwo);

    public AlgaeBarIOSpark(){
        //could do motor configuration here
    }

    @Override
    public void updateInputs(AlgaeBarIOInputs inputs) {
        ifOk(rotateMotor, rotateEncoder::getPosition, (value) -> inputs.rotateMotorPositionRad = value);  
        ifOk(rotateMotor, rotateEncoder::getVelocity, (value) -> inputs.rotateMotorVelocityRadPerSec = value);
        ifOk(
            rotateMotor,
                new DoubleSupplier[] {rotateMotor::getAppliedOutput, rotateMotor::getBusVoltage},
                (values) -> inputs.rotateMotorAppliedVolts = values[0] * values[1]);
        ifOk(rotateMotor, rotateMotor::getOutputCurrent, (value) -> inputs.rotateMotorCurrentAmps = value);   
        
        ifOk(coralMotor, coralEncoder::getPosition, (value) -> inputs.coralMotorPositionRad = value); 
        ifOk(coralMotor, coralEncoder::getVelocity, (value) -> inputs.coralMotorVelocityRadPerSec = value);
        ifOk(
            coralMotor,
                new DoubleSupplier[] {coralMotor::getAppliedOutput, coralMotor::getBusVoltage},
                (values) -> inputs.coralMotorAppliedVolts = values[0] * values[1]);
        ifOk(coralMotor, coralMotor::getOutputCurrent, (value) -> inputs.coralMotorCurrentAmps= value); 
    } 

    @Override
    public void setVoltage(double volts) {
            rotateMotor.setVoltage(volts);
            coralMotor.setVoltage(volts);
        }
 }