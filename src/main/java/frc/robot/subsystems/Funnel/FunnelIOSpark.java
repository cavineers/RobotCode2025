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
    public DigitalInput funnelSensor1 = new DigitalInput(FunnelConstants.funnelSensor1);


    public FunnelIOSpark(){
        
    }


    @Override
    public void setVoltage(double volts) {
        funnelMotor.setVoltage(volts);
    }

    private boolean isSensorHit(DigitalInput sensor) {
            return sensor.get(); //put in actual threshold
        }

    public void updateInputs(FunnelIOInputs inputs) {
           if (isSensorHit(funnelSensor1)) {
            inputs.velocityRadPerSec = 0;
    } else {
        ifOk(funnelMotor, encoder::getPosition,(value) -> inputs.positionRad = value);
        ifOk(funnelMotor, encoder::getVelocity,(value) -> inputs.velocityRadPerSec = value);
        ifOk(
            funnelMotor,
                new DoubleSupplier[] {funnelMotor::getAppliedOutput, funnelMotor::getBusVoltage},
                (values) -> inputs.appliedVolts = values[0] * values[1]);
        }
    }
}