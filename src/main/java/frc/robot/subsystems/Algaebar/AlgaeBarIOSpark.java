package frc.robot.subsystems.Algaebar;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.Algaebar.AlgaeBarConstants.kAlgaeBarRotateCanID;
import static frc.robot.subsystems.Algaebar.AlgaeBarConstants.kAlgaeBarCoralCanID;


import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class AlgaeBarIOSpark implements AlgaeBarIO {
    private final SparkMax rotateMotor = new SparkMax(kAlgaeBarRotateCanID, MotorType.kBrushless);
    private final RelativeEncoder rotateEncoder = rotateMotor.getEncoder();

    private final SparkMax coralMotor = new SparkMax(kAlgaeBarCoralCanID, MotorType.kBrushless);
    private final RelativeEncoder coralEncoder = coralMotor.getEncoder();

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
    } 
    @Override
    public void setVoltage(double volts) {
            rotateMotor.setVoltage(volts);
        }
    }