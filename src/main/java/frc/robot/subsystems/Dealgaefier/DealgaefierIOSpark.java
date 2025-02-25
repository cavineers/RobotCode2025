package frc.robot.subsystems.Dealgaefier;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.Dealgaefier.DealgaefierConstants.kDeployCanID;
import static frc.robot.subsystems.Dealgaefier.DealgaefierConstants.kIntakeCanID;
import static frc.robot.subsystems.Dealgaefier.DealgaefierConstants.kProportionalGainSpark;
import static frc.robot.subsystems.Dealgaefier.DealgaefierConstants.kIntegralTermSpark;
import static frc.robot.subsystems.Dealgaefier.DealgaefierConstants.kDerivativeTermSpark;
import static frc.robot.subsystems.Dealgaefier.DealgaefierConstants.kGravityTermSpark;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class DealgaefierIOSpark implements DealgaefierIO {
    final SparkMax deployMotor = new SparkMax(kDeployCanID, MotorType.kBrushless);
    final RelativeEncoder deployEncoder = deployMotor.getEncoder();

    final SparkMax intakeMotor = new SparkMax(kIntakeCanID, MotorType.kBrushless);
    final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    public DutyCycleEncoder deployAbsEncoder = new DutyCycleEncoder(DealgaefierConstants.kDeployAbsEncoder);

    private LoggedNetworkNumber tuningP = new LoggedNetworkNumber("/Tuning/Dealgaefier/P", DealgaefierConstants.kProportionalGainSpark);
    private LoggedNetworkNumber tuningD = new LoggedNetworkNumber("/Tuning/Dealgaefier/D", DealgaefierConstants.kDerivativeTermSpark);
    private LoggedNetworkNumber tuningG = new LoggedNetworkNumber("/Tuning/Dealgaefier/G", DealgaefierConstants.kGravityTermSpark); 

    private double absSetpoint;

    private PIDController controller = new PIDController(kProportionalGainSpark, kIntegralTermSpark, kDerivativeTermSpark);

    public boolean absEncoderInitialized = false;
    
    public DealgaefierIOSpark(){

    }

    @Override
    public void updateInputs(DealgaefierIOInputs inputs) {

        ifOk(deployMotor, deployEncoder::getPosition, (value) -> inputs.deployMotorPositionRotations = value);
        ifOk(deployMotor, deployEncoder::getVelocity, (value) -> inputs.deployMotorVelocityRadPerSec = value);
        ifOk(
            deployMotor,
            new DoubleSupplier[] {deployMotor::getAppliedOutput, deployMotor::getBusVoltage},
            (values -> inputs.deployMotorAppliedVolts = values[0] * values[1]));
        ifOk(deployMotor, deployMotor::getOutputCurrent, (value) -> inputs.deployMotorCurrentAmps = value);
        
        ifOk(intakeMotor, intakeEncoder::getPosition, (value) -> inputs.intakeMotorPositionRotations = value);
        ifOk(intakeMotor, intakeEncoder::getVelocity, (value) -> inputs.intakeMotorVelocityRadPerSec = value);
        ifOk(
            intakeMotor,
            new DoubleSupplier[] {intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage},
            (values -> inputs.intakeMotorAppliedVolts = values[0] * values[1]));
        ifOk(intakeMotor, intakeMotor::getOutputCurrent, (value) -> inputs.intakeMotorCurrentAmps = value);

        if(absEncoderInitialized == false) {
            initializeDutyEncoder();
            System.out.println("fort");
        }

        double desiredVoltage = this.controller.calculate(getAbsEncoder()) + this.tuningG.get();
        this.setDeployVoltage(desiredVoltage);
        System.out.println(desiredVoltage);

        if (DealgaefierConstants.kTuningMode){
            this.updatePID();
        }
    }

    public void initializeDutyEncoder(){
        this.absSetpoint = getAbsEncoder();
        absEncoderInitialized = true;
    }

    public boolean getSensor(DigitalInput sensor) {
        return sensor.get();
    }

    public double getAbsEncoder() {
        return this.deployAbsEncoder.get();
    }
    
    @Override
    public void setDeployVoltage(double volts) {
        deployMotor.setVoltage(volts);
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }

    public void updateSetpoint(double setpoint) {
        this.absSetpoint = this.clipSetpoint(setpoint);
        this.controller.setSetpoint(setpoint);
    }

    public double clipSetpoint(double setpoint) {
        if(absSetpoint > DealgaefierConstants.kDeployedAbsoluteRotations) {
            return DealgaefierConstants.kDeployedAbsoluteRotations;
        } else if(absSetpoint < DealgaefierConstants.kRestAbsoluteRotations) {
            return DealgaefierConstants.kRestAbsoluteRotations;
        }
        return setpoint;
    }

    private void updatePID(){
        double currentP = this.controller.getP();
        double currentD = this.controller.getD();

        if (currentP != this.tuningP.get() || currentD != this.tuningD.get()){
            this.controller.setPID(this.tuningP.get(), 0, this.tuningD.get());
        }
    }
}
