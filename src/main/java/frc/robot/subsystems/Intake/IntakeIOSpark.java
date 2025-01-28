package frc.robot.subsystems.Intake;

import static frc.robot.subsystems.Intake.IntakeConstants.kLeftIntakeCanID;
import static frc.robot.subsystems.Intake.IntakeConstants.kRightIntakeCanID; 

import static frc.lib.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkMax;
    
    
    
    
    public class IntakeIOSpark implements IntakeIO {
        private final SparkMax leftMotor = new SparkMax(kLeftIntakeCanID, MotorType.kBrushless);
        private final SparkMax rightMotor = new SparkMax(kRightIntakeCanID, MotorType.kBrushless);
        private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
        private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
        public DigitalInput leftSensor = new DigitalInput(IntakeConstants.kCoralSensorLeft);
        public DigitalInput rightSensor = new DigitalInput(IntakeConstants.kCoralSensorRight);

    
        public IntakeIOSpark(){
            //motor config 
        }
    
   
        
        
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
               leftMotor.setVoltage(0);
               rightMotor.setVoltage(0);
    } else {
            ifOk(leftMotor, leftEncoder::getPosition, value -> inputs.positionRad = value);
            ifOk(rightMotor, rightEncoder::getPosition, value -> inputs.positionRad = value); 
    }

    
}
}     