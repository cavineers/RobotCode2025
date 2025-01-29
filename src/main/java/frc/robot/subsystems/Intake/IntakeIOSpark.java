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

    public void updateInputs(IntakeIOInputs inputs) {
        ifOk(leftMotor, leftEncoder::getPosition, value -> inputs.positionRadLeft = value);
        ifOk(rightMotor, rightEncoder::getPosition, value -> inputs.positionRadRight = value); 
        ifOk(leftMotor, leftEncoder::getPosition, value -> inputs.appliedVoltsLeft = value);
        ifOk(rightMotor, rightEncoder::getPosition, value -> inputs.appliedVoltsRight = value);
        ifOk(leftMotor, leftEncoder::getPosition, value -> inputs.currentAmpsLeft = value);
        ifOk(rightMotor, rightEncoder::getPosition, value -> inputs.currentAmpsRight = value);
        ifOk(leftMotor, leftEncoder::getPosition, value -> inputs.velocityRadPerSecLeft = value);
        ifOk(rightMotor, rightEncoder::getPosition, value -> inputs.velocityRadPerSecRight = value);
        inputs.leftSensor = leftSensor.get();
        inputs.rightSensor = rightSensor.get();
    }

    
}
     