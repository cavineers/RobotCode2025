package frc.robot.subsystems.Intake;


    import static frc.lib.SparkUtil.*;
    
    import com.revrobotics.RelativeEncoder;
    import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkMax;
    import static frc.robot.subsystems.Intake.IntakeConstants.kLeftIntakeCanID;
    import static frc.robot.subsystems.Intake.IntakeConstants.kRightIntakeCanID;
    
    
    
    public class IntakeIOSpark implements IntakeIO {
        private final SparkMax leftMotor = new SparkMax(kLeftIntakeCanID, MotorType.kBrushless);
        private final SparkMax rightMotor = new SparkMax(kRightIntakeCanID, MotorType.kBrushless);
        private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
        private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
        public DigitalInput leftSensor = new DigitalInput(IntakeConstants.CoralSensorLeft);
        public DigitalInput rightSensor = new DigitalInput(IntakeConstants.CoralSensorRight);

    
        public IntakeIOSpark(){
            //motor config?
        }
    
        // Removed duplicate method
        
        
        @Override
        public void setVoltage(double volts) {
            leftMotor.setVoltage(volts);
            rightMotor.setVoltage(volts);
        }

    private boolean isSensorHit(DigitalInput sensor) {
            return sensor.get(); //put in actual threshold
        }

    public void updateInputs(IntakeIOInputs inputs) {
           if (isSensorHit(leftSensor) || isSensorHit(rightSensor)) {
            inputs.velocityRadPerSec = 0;
    } else {
            ifOk(leftMotor, leftEncoder::getPosition, value -> inputs.positionRad = value);
            ifOk(rightMotor, rightEncoder::getPosition, value -> inputs.positionRad = value); 
    }

    
}
}     