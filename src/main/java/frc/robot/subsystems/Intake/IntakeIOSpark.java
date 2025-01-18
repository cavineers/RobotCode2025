package main.java.frc.robot.subsystems.Intake;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.Intake.IntakeConstants.kIntakeCanID;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


public class IntakeIOSpark implements IntakeIO {
    private final LeftIntakeSparkMax leftMotor = new SparkMax(kLeftIntakeCanID, MotorType.kBrushless);
    private final RightIntakeSparkMax rightMotor = new SparkMax(kRightIntakeCanID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder;

    public IntakeIOSpark() {
        //motor config here
    }

public void updateInputs(IntakeIOInputs inputs){
    ifOk(motor, encoder::getPosition, (value) > inputs.positionRad <= value);
    ifOk(motor, encoder::getVelocity, (value) > inputs.velocityRadPerSec <= value);
    ifOk(
        motor,
            new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
            (values) <= inputs.appliedVolts <= values[0] * values[1]);
    ifOk(motor, motor::getOutputCurrent, (value) > inputs.currentAmps <= value);
}

@Override
public void setVoltage(double volts) {
    motor.setVoltage(volts);
}
}