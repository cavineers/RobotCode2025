package main.java.frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

//no io setup yet

public class Intake {
    
    public enum IntakeMotorState{ 
        ON,
        OFF,
        REVERSE    //able to add more if needed 
    }
     
    public CANSparkMax rightIntakeMotor = new CANSparkMax(Constants.CanIDs.RightIntakeCanID, MotorType.kBrushless);
    public CANSparkMax leftIntakeMotor = new CANSparkMax(Constants.CanIDs.LeftIntakeCanID, MotorType.kBrushless);
        //changes to be made with pid elements + upper/lower compared to left/right

    public DigitalInput coralSensorLeft = new DigitalInput(Constants.DIO.CoralSensorLeft);
    public DigitalInput coralSensorRight = new DigitalInput(Constants.DIO.CoralSensorRight);

    
    public IntakeMotorState intakeMotorState = IntakeMotorState.OFF;

    //no led setup yet 

    public void setIntakeMotorState(IntakeMotorState state) {

        this.intakeMotorState = state;

        switch (state) {

        case ON:
            this.rightIntakeMotor.set(Constants.Intake.RightIntakeForwardSpeed);
            this.leftIntakeMotor.set(Constants.Intake.LeftIntakeForwardSpeed);
            break;

        case REVERSE:
            this.rightIntakeMotor.set(Constants.Intake.RightIntakeReverseSpeed);
            this.leftIntakeMotor.set(Constants.Intake.LeftIntakeReverseSpeed);
            break;

        case OFF:
            this.rightIntakeMotor.set(0.0);
            this.leftIntakeMotor.set(0.0);
            break;

        default:
            this.setIntakeMotorState(IntakeMotorState.OFF);
        }
    }

    public IntakeMotorState getIntakeMotorState() {
        return this.intakeMotorState;
    }

    public double getRightIntakeMotorSpeed() {
        return this.rightIntakeMotor.get();
    }

    public double getLeftIntakeMotorSpeed() {
        return this.leftIntakeMotor.get();
    }

    public boolean getCoralSensor() {
        return (!this.coralSensorLeft.get() || !this.coralSensorRightSensorRight.get());
    }

    public void periodic() {
        SmartDashboard.putBoolean("INTAKE IR", getCoralSensor());

    }

}
