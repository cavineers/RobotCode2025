package frc.robot.subsystems.Intake;


    import java.util.function.DoubleSupplier;
    import static frc.lib.SparkUtil.*;
    
    import com.revrobotics.RelativeEncoder;
    import com.revrobotics.spark.SparkLowLevel.MotorType;
    import com.revrobotics.spark.SparkMax;
    import static frc.robot.subsystems.Intake.IntakeConstants.kLeftIntakeCanID;
    import static frc.robot.subsystems.Intake.IntakeConstants.kRightIntakeCanID;
    
    
    
    public class IntakeIOSpark implements IntakeIO {
        private final SparkMax leftMotor = new SparkMax(kLeftIntakeCanID, MotorType.kBrushless);
        private final SparkMax rightMotor = new SparkMax(kRightIntakeCanID, MotorType.kBrushless);
        private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
        private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    
        public IntakeIOSpark(){
            //motor config?
        }
    
        public void updateInputs(IntakeIOInputs inputs){
            ifOk(leftMotor, leftEncoder::getPosition, value -> inputs.positionRad = value);
            ifOk(rightMotor, rightEncoder::getVelocity, value -> inputs.velocityRadPerSec = value);
            ifOk(
            leftMotor,
                new DoubleSupplier[] {leftMotor::getAppliedOutput, leftMotor::getBusVoltage},
                (values) -> inputs.appliedVolts = values[0] * values[1]);
                ifOk(leftMotor, leftMotor::getOutputCurrent, (value) -> inputs.currentAmps = value); 
            ifOk(
            rightMotor,
                 new DoubleSupplier[] {rightMotor::getAppliedOutput, rightMotor::getBusVoltage},
                (values) -> inputs.appliedVolts = values[0] * values[1]);
                ifOk(rightMotor, rightMotor::getOutputCurrent, (value) -> inputs.currentAmps = value);          
        }
        
        
        @Override
        public void setVoltage(double volts) {
            leftMotor.setVoltage(volts);
            rightMotor.setVoltage(volts);
        }
        }
        
        