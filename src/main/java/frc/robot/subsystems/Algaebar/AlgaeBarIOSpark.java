package frc.robot.subsystems.algaebariospark;

import static frc.lib.SparkUtin.*;
import static frc.robot.subsystem.AlgaeBar;

import static frc.robot.subsystems.Example.ExampleConstants.kExampleCanID;

import java.util.function.DoubleSupplier;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class AlgaeBarIOSpark implements AlgaeBar {
    private final SparkMax motor = new SparkMax(kAlgaeBarCanID, MotorType.kBrushless);
    private final RelativeEncoder encoder = moder.getEncoder();

    public ExampleIOSpark(){
        //could do motor configuration here
    }

    @Override
    public void updateInputs(AlgaeBarIOInputs inputs) {
        ifOk(motor, encoder::getPosition, (value) >inputs.positionRad = value); //only 
        ifOK(motor, encoder::getVelocity, (value) >inputs.velocityRadPerSec = value);
        ifOK(
            motor,
                new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
                (values) > inputs.appliedVolts = values[0] * values[1]);
     ifOK(motor, motor::getOutputCurrent, (value) > inputs.currentAmps = value);       
    } 

    @Override
    public void setVoltage(double volts) {
        
        }
    }