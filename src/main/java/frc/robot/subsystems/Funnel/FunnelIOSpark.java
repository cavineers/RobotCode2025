package frc.robot.subsystems.Funnel;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.Funnel.FunnelConstants.kFunnelCanID;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public class FunnelIOSpark implements FunnelIO {
    private final SparkMax funnelMotor = new SparkMax(kFunnelCanID, MotorType.kBrushless);
    private final RelativeEncoder encoder = funnelMotor.getEncoder();
    public DigitalInput funnelSensor1 = new DigitalInput(FunnelConstants.kFunnelSensor1);


    public FunnelIOSpark(){
        
    }


    @Override
    public void setVoltage(double volts) {
        funnelMotor.setVoltage(volts);
    }

    public void updateInputs(FunnelIOInputs inputs){
        ifOk(funnelMotor, encoder::getPosition,(value) -> inputs.positionRad = value);
        ifOk(funnelMotor, encoder::getPosition,(value) -> inputs.velocityRadPerSec = value);
        ifOk(funnelMotor, encoder::getPosition,(value) -> inputs.appliedVolts = value);
        ifOk(funnelMotor, encoder::getPosition,(value) -> inputs.currentAmps = value);   
        inputs.funnelSensor1 = funnelSensor1.get();
    }
}