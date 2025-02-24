package frc.robot.subsystems.Algaebar;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.Algaebar.AlgaeBarConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AlgaeBarIOSpark implements AlgaeBarIO {
    private final SparkMax pivotMotor = new SparkMax(kAlgaeBarPivotCanID, MotorType.kBrushless); 
    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    private final SparkMax algaeBarMotor = new SparkMax(kAlgaeBarCanID, MotorType.kBrushless);
    private final RelativeEncoder algaeBarEncoder = algaeBarMotor.getEncoder(); 

    public DutyCycleEncoder algaeBarAbsEncoder = new DutyCycleEncoder(AlgaeBarConstants.kAlgaeBarAbsEncoder);

    public double motorSetpoint;


    public AlgaeBarIOSpark(){
        //could do motor configuration here
    }

    @Override
    public void updateInputs(AlgaeBarIOInputs inputs) {
        ifOk(pivotMotor, pivotEncoder::getPosition, (value) -> inputs.pivotMotorPositionRad = value);  
        ifOk(pivotMotor, pivotEncoder::getVelocity, (value) -> inputs.pivotMotorVelocityRadPerSec = value);
        ifOk(
            pivotMotor,
                new DoubleSupplier[] {pivotMotor::getAppliedOutput, pivotMotor::getBusVoltage},
                (values) -> inputs.pivotMotorAppliedVolts = values[0] * values[1]);
        ifOk(pivotMotor, pivotMotor::getOutputCurrent, (value) -> inputs.pivotMotorCurrentAmps = value);   
        
        ifOk(algaeBarMotor, algaeBarEncoder::getPosition, (value) -> inputs.algaeBarMotorPositionRad = value); 
        ifOk(algaeBarMotor, algaeBarEncoder::getVelocity, (value) -> inputs.algaeBarMotorVelocityRadPerSec = value);
        ifOk(
            algaeBarMotor,
                new DoubleSupplier[] {algaeBarMotor::getAppliedOutput, algaeBarMotor::getBusVoltage},
                (values) -> inputs.algaeBarMotorAppliedVolts = values[0] * values[1]);
        ifOk(algaeBarMotor, algaeBarMotor::getOutputCurrent, (value) -> inputs.algaeBarMotorCurrentAmps= value); 
    } 

    public void initizlizeDutyEncoder(){
        this.motorSetpoint = algaeBarAbsEncoder.get();
    }  

    @Override
    public void setPivotVoltage(double volts) {
            pivotMotor.setVoltage(volts);
        }

    @Override
    public void setAlgaeBarVoltage(double volts) {
        algaeBarMotor.setVoltage(volts);
    }
 }