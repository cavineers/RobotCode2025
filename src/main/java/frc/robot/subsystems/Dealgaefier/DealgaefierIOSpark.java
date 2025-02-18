package frc.robot.subsystems.Dealgaefier;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.Dealgaefier.DealgaefierConstants.kDealgaefierSpinCanID;
import static frc.robot.subsystems.Dealgaefier.DealgaefierConstants.kDealgaefierPivotCanID;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public class DealgaefierIOSpark implements DealgaefierIO {
    final SparkMax pivotMotor = new SparkMax(kDealgaefierPivotCanID, MotorType.kBrushless);
    final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    final SparkMax spinMotor = new SparkMax(kDealgaefierSpinCanID, MotorType.kBrushless);
    final RelativeEncoder spinEncoder = spinMotor.getEncoder();

    private final DigitalInput dealgaefierLimit = new DigitalInput(DealgaefierConstants.kDealgaefierLimit);
    public DealgaefierIOSpark(){
        //motor config potentially
    }

    
    @Override
    public void updateInputs(DealgaefierIOInputs inputs) {
        ifOk(pivotMotor, pivotEncoder::getPosition, (value) -> inputs.pivotMotorPositionRad = value);
        ifOk(pivotMotor, pivotEncoder::getVelocity, (value) -> inputs.pivotMotorVelocityRadPerSec = value);
        ifOk(
            pivotMotor,
            new DoubleSupplier[] {pivotMotor::getAppliedOutput, pivotMotor::getBusVoltage},
            (values -> inputs.pivotMotorAppliedVolts = values[0] * values[1]));
        ifOk(pivotMotor, pivotMotor::getOutputCurrent, (value) -> inputs.pivotMotorCurrentAmps = value);
        
        ifOk(spinMotor, spinEncoder::getPosition, (value) -> inputs.spinMotorPositionRad = value);
        ifOk(spinMotor, spinEncoder::getVelocity, (value) -> inputs.spinMotorVelocityRadPerSec = value);
        ifOk(
            spinMotor,
            new DoubleSupplier[] {spinMotor::getAppliedOutput, spinMotor::getBusVoltage},
            (values -> inputs.spinMotorAppliedVolts = values[0] * values[1]));
        ifOk(pivotMotor, pivotMotor::getOutputCurrent, (value) -> inputs.spinMotorCurrentAmps = value);
    }

    public boolean getSensor(DigitalInput sensor) {
        return sensor.get();
    }
    
    @Override
    public void setVoltage(double volts) {
        pivotMotor.setVoltage(volts);
        spinMotor.setVoltage(volts);
    }

    public void pivot() {
        if(getSensor(dealgaefierLimit) == false) {
            setVoltage(DealgaefierConstants.kDealgaefierPivotSpeed);
        }
    }

}
