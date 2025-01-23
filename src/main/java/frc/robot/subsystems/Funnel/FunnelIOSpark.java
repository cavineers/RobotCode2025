package frc.robot.subsystems.Funnel;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.Funnel.FunnelConstants.kFunnelCanID;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class FunnelIOSpark implements FunnelIO {
    private final SparkMax motor = new SparkMax(kFunnelCanID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();

    public FunnelIOSpark(){
        
    }

    @Override
    public void updateInputs(FunnelIOInputs inputs){
        ifOk(motor, encoder::getPosition,(value) -> inputs.positionRad = value);
        ifOk(motor, encoder::getVelocity,(value) -> inputs.velocityRadPerSec = value);
        ifOk(
            motor,
                new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
                (values) -> inputs.appliedVolts = values[0] * values[1]);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}