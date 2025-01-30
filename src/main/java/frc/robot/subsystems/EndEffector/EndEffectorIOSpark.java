package frc.robot.subsystems.EndEffector;

import static frc.lib.SparkUtil.*;
import static frc.robot.subsystems.EndEffector.EndEffectorConstants.kLeftEndEffectorCanID;
import static frc.robot.subsystems.EndEffector.EndEffectorConstants.kRightEndEffectorCanID;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkMax;
    
    
    
    
    public class EndEffectorIOSpark implements EndEffectorIO {
        private final SparkMax leftMotor = new SparkMax(kLeftEndEffectorCanID, MotorType.kBrushless);
        private final SparkMax rightMotor = new SparkMax(kRightEndEffectorCanID, MotorType.kBrushless);
        private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
        private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
        public DigitalInput leftSensor = new DigitalInput(EndEffectorConstants.kCoralSensorLeft);
        public DigitalInput rightSensor = new DigitalInput(EndEffectorConstants.kCoralSensorRight);

    
        public EndEffectorIOSpark(){
            //motor config 
        }
    
   
        
        
        @Override
        public void setVoltage(double volts) {
            leftMotor.setVoltage(volts);
            rightMotor.setVoltage(volts);
        }

    public void updateInputs(EndEffectorIOInputs inputs) {
        ifOk(leftMotor, leftEncoder::getPosition, value -> inputs.leftPositionRad = value);
        ifOk(rightMotor, rightEncoder::getPosition, value -> inputs.rightPositionRad = value); 
        ifOk(leftMotor, leftEncoder::getPosition, value -> inputs.leftAppliedVolts = value);
        ifOk(rightMotor, rightEncoder::getPosition, value -> inputs.rightAppliedVolts = value);
        ifOk(leftMotor, leftEncoder::getPosition, value -> inputs.leftCurrentAmps = value);
        ifOk(rightMotor, rightEncoder::getPosition, value -> inputs.rightCurrentAmps = value);
        ifOk(leftMotor, leftEncoder::getPosition, value -> inputs.leftVelocityRadPerSec = value);
        ifOk(rightMotor, rightEncoder::getPosition, value -> inputs.rightVelocityRadPerSec = value);
        inputs.leftSensor = leftSensor.get();
        inputs.rightSensor = rightSensor.get();
    }

    
}
     