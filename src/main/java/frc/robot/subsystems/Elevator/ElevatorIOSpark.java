package frc.robot.subsystems.Elevator;

import static frc.lib.SparkUtil.*; // has a bunch of utility functions for SparkMax

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorState;

public class ElevatorIOSpark implements ElevatorIO {
    
    private final SparkFlex rightMotor = new SparkFlex(kRightMotorCanID, MotorType.kBrushless);
    private final SparkFlex leftMotor = new SparkFlex(kLeftMotorCanID, MotorType.kBrushless);

    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

    private LoggedNetworkNumber tuningP = new LoggedNetworkNumber("/Tuning/Elevator/P", ElevatorConstants.kProportionalGainSpark);
    private LoggedNetworkNumber tuningD = new LoggedNetworkNumber("/Tuning/Elevator/D", ElevatorConstants.kDerivativeTermSpark);
    private LoggedNetworkNumber tuningG = new LoggedNetworkNumber("/Tuning/Elevator/G", ElevatorConstants.kGravityTermSpark); 

    private LoggedNetworkNumber tuningMaxVelocity = new LoggedNetworkNumber("/Tuning/Elevator/MaxVelocity", ElevatorConstants.kMaxVelocityRPS);
    private LoggedNetworkNumber tuningMaxAcceleration = new LoggedNetworkNumber("/Tuning/Elevator/MaxAcceleration", ElevatorConstants.kMaxAccelerationRPS2);
    private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchID);

    @AutoLogOutput(key = "Elevator/Goal")
    private double motorSetpoint = 0;

    private SparkFlexConfig config;

    private Constraints motionConstraints = new Constraints(kMaxVelocityRPS, kMaxAccelerationRPS2);
    private ProfiledPIDController controller = new ProfiledPIDController(kProportionalGainSpark, kIntegralTermSpark, kDerivativeTermSpark, motionConstraints);

    @AutoLogOutput(key="Elevator/IsClosed")
    private boolean isClosed = true;

    public ElevatorIOSpark() {
        // Set up the PID controller on Spark Max
        config = new SparkFlexConfig();
        config
            .inverted(kInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ElevatorConstants.kCurrentLimit)    
            .voltageCompensation(12);
        config.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);   
     
        tryUntilOk(
            rightMotor,
            5,
            () -> rightMotor.configure(config, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));

        var leftMotorConfig = new SparkFlexConfig().apply(config).idleMode(IdleMode.kBrake);
        leftMotorConfig.follow(rightMotor);
        tryUntilOk(
            leftMotor,
            5,
            () -> leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));

        this.controller.setTolerance(kTolerance);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // if (limitSwitch.get()){
        //     this.resetPosition();
        // }
        // Update for right motor
        ifOk(rightMotor, rightEncoder::getPosition, (value) -> inputs.rightPositionRotations = value);
        ifOk(rightMotor, rightEncoder::getVelocity, (value) -> inputs.rightVelocityRPM = value);
        ifOk(
            rightMotor,
            new DoubleSupplier[] {rightMotor::getAppliedOutput, rightMotor::getBusVoltage},
            (values) -> inputs.rightAppliedVolts = values[0] * values[1]);        
        
        ifOk(rightMotor, rightMotor::getOutputCurrent, (value) -> inputs.rightCurrentAmps = value);

        ifOk(leftMotor, leftEncoder::getPosition, (value) -> inputs.leftPositionRotations = value);
        ifOk(leftMotor, leftEncoder::getVelocity, (value) -> inputs.leftVelocityRPM = value);
        ifOk(
            leftMotor,
            new DoubleSupplier[] {leftMotor::getAppliedOutput, leftMotor::getBusVoltage},
            (values) -> inputs.leftAppliedVolts = values[0] * values[1]);        
        
        ifOk(leftMotor, leftMotor::getOutputCurrent, (value) -> inputs.leftCurrentAmps = value);


        // Update limit switch
        inputs.limitSwitch = getLimitSwitch(); 
        
        // Update setpoint
        inputs.setpoint = motorSetpoint;
        
        double desiredVoltage = this.controller.calculate(inputs.rightPositionRotations) + this.calculateFeedforward();
        if (desiredVoltage > 6.0){
            desiredVoltage = 6.0;
        } else if (desiredVoltage < -5.0){
            desiredVoltage = -5.0;
        }

        Logger.recordOutput("Elevator/PIDRequestedVoltage", desiredVoltage);
        Logger.recordOutput("Elevator/OutputCurrent", rightMotor.getAppliedOutput());
        Logger.recordOutput("Elevator/ProfiledSetpoint", this.controller.getSetpoint().position);

        if (this.isClosed){
            this.setVoltage(desiredVoltage);
        }

        if (kTuningMode){
            this.updatePID();
            this.updateMotionProfiling();
        }

        if (controller.atGoal()) {
            inputs.state = switch ((int) motorSetpoint) {
            case (int) ElevatorConstants.kRestRotations -> ElevatorState.REST;
            case (int) ElevatorConstants.kL1Rotations -> ElevatorState.L1;
            case (int) ElevatorConstants.kL2Rotations -> ElevatorState.L2;
            case (int) ElevatorConstants.kL3Rotations -> ElevatorState.L3;
            case (int) ElevatorConstants.kL4Rotations -> ElevatorState.L4; 
            default -> ElevatorState.REST;
            };
        }
    }

    private double calculateFeedforward() {
        double feedforward = ElevatorConstants.kTuningMode ? this.tuningG.get() : ElevatorConstants.kGravityTermSpark;
        // if (this.rightMotor.getEncoder().getPosition() < ElevatorConstants.kGravityTermChangeRotations){
        //     return feedforward;
        // }
        return feedforward + ElevatorConstants.kGravityTermHeightCompensation;
    }

    public double getElevMotorPosition() {
        return rightEncoder.getPosition();
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    public void setVoltage(double volts) {
        rightMotor.setVoltage(volts);
    }
    
    public void resetPosition() {
        rightEncoder.setPosition(0);
        leftEncoder.setPosition(0);
    }

    public void updateSetpoint(double setpoint) {
        this.motorSetpoint = this.clipSetpoint(setpoint);
        this.controller.setGoal(motorSetpoint);
    }

    public double clipSetpoint(double setpoint) {
        if(motorSetpoint > ElevatorConstants.kMaxRotations) {
            return ElevatorConstants.kMaxRotations;
        } else if(motorSetpoint < ElevatorConstants.kMinRotations) {
            return ElevatorConstants.kMinRotations;
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

    private void updateMotionProfiling(){
        double currentMaxVelocity = this.controller.getConstraints().maxVelocity;
        double currentMaxAcceleration = this.controller.getConstraints().maxAcceleration;

        if (currentMaxVelocity != tuningMaxAcceleration.get() || currentMaxAcceleration != tuningMaxAcceleration.get()){
            this.controller.setConstraints(new Constraints(tuningMaxVelocity.get(),  tuningMaxAcceleration.get()));
        }
    }

    @Override
    public void setClosedLoop(boolean val) {
        if (val != this.isClosed){
            this.controller.reset(this.rightEncoder.getPosition());
        }
        this.isClosed = val;
    }

    @Override
    @AutoLogOutput(key = "Elevator/Error")
    public double getError() {
        return controller.getPositionError();
    }
}