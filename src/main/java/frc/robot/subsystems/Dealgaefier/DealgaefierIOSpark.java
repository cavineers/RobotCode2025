package frc.robot.subsystems.Dealgaefier;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.Dealgaefier.DealgaefierConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.littletonrobotics.junction.AutoLogOutput;

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

    @AutoLogOutput(key="Dealgaefier/Setpoint")
    private double absSetpoint;

    private PIDController controller = new PIDController(kProportionalGainSpark, kIntegralTermSpark, kDerivativeTermSpark);

    private SparkMaxConfig deployConfig;

    @AutoLogOutput(key="Dealgaefier/DesiredVoltage")
    private double desiredVoltage = 0.0;

    public boolean absEncoderInitialized = false;

    public boolean deployed = false;
    
    public DealgaefierIOSpark(){
        this.controller.enableContinuousInput(0, 1);
    
        deployConfig = new SparkMaxConfig();
        deployConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DealgaefierConstants.kCurrentLimit)    
            .voltageCompensation(12); 
     
        tryUntilOk(
            deployMotor,
            5,
            () -> deployMotor.configure(deployConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));

        var intakeConfig = new SparkMaxConfig().apply(deployConfig);
        
        intakeConfig.inverted(true);
        tryUntilOk(
            intakeMotor,
            5,
            () -> intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));

        this.controller.setTolerance(kTolerance); // doesn't actually do anything unless you are using controller.atSetpoint()

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
        }

        this.desiredVoltage = this.controller.calculate(getAbsEncoder()) * -1.0 + (this.tuningG.get() * Math.cos(getAbsEncoder()*2*Math.PI));
        this.setDeployVoltage(this.desiredVoltage);

        if (DealgaefierConstants.kTuningMode){
            this.updatePID();
        }
    }

    public void initializeDutyEncoder(){
        this.absSetpoint = getAbsEncoder();
        this.controller.setSetpoint(absSetpoint);
        absEncoderInitialized = true;
    }

    public boolean getSensor(DigitalInput sensor) {
        return sensor.get();
    }

    @AutoLogOutput(key="Dealgaefier/AbsRotations")
    public double getAbsEncoder() {
        return this.deployAbsEncoder.get() + kAbsEncoderOffset;
    }

    public double getDeployPositionRotations() {
        return deployEncoder.getPosition();
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
        this.controller.setSetpoint(absSetpoint);
    }

    public double clipSetpoint(double setpoint) {
        // if(absSetpoint < DealgaefierConstants.kDeployedAbsoluteRotations) {
        //     return DealgaefierConstants.kDeployedAbsoluteRotations;
        // } else if(absSetpoint < DealgaefierConstants.kRestAbsoluteRotations) {
        //     return DealgaefierConstants.kRestAbsoluteRotations;
        // }
        return setpoint;
    }

    private void updatePID() {
        double currentP = this.controller.getP();
        double currentD = this.controller.getD();

        if (currentP != this.tuningP.get() || currentD != this.tuningD.get()){
            this.controller.setPID(this.tuningP.get(), 0, this.tuningD.get());
        }
    }

    public void deploy() {
        if(deployed == false) {
            updateSetpoint(kDeployedAbsoluteRotations);
            setIntakeVoltage(kIntakeSpeed * 12.0);
        } else {
            updateSetpoint(kRestAbsoluteRotations);
            setIntakeVoltage(0.0);
        }
        deployed = !deployed;
    }

    public void shoot() {
        updateSetpoint(kDeployedAbsoluteRotations);
        setIntakeVoltage(kIntakeSpeed * -12.0);
    }

    public void retract() {
        updateSetpoint(kRestAbsoluteRotations);
        setIntakeVoltage(0.0);
    }
 }
