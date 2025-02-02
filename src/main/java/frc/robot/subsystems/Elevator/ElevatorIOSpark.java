package frc.robot.subsystems.Elevator;

import static frc.lib.SparkUtil.*; // has a bunch of utility functions for SparkMax

import static frc.robot.subsystems.Elevator.ElevatorConstants.kLeftMotorCanID;
import static frc.robot.subsystems.Elevator.ElevatorConstants.kRightMotorCanID;
import static frc.robot.subsystems.Elevator.ElevatorConstants.kTolerance;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOSpark implements ElevatorIO {
    
    private final SparkFlex rightMotor = new SparkFlex(kLeftMotorCanID, MotorType.kBrushless);
    private final SparkFlex leftMotor = new SparkFlex(kRightMotorCanID, MotorType.kBrushless);

    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

    private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchID);

    @AutoLogOutput(key = "Elevator/Setpoint")
    private double motorSetpoint = 0;

    public ElevatorIOSpark() {
        // Set up the PID controller on Spark Max
        SparkFlexConfig config = new SparkFlexConfig();
        config
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12)
            .smartCurrentLimit(ElevatorConstants.kCurrentLimit);        
        config.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(ElevatorConstants.kProportionalGainSpark, ElevatorConstants.kIntegralTermSpark, ElevatorConstants.kDerivativeTermSpark, 0);
        
        tryUntilOk(
            rightMotor,
            5,
            () -> rightMotor.configure(config, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));

        config.follow(rightMotor); // Set the left motor to follow the right motor (might want to lower telemetry refresh rates in the future)
        tryUntilOk(
            leftMotor,
            5,
            () -> leftMotor.configure(config, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Update motor inputs
        ifOk(leftMotor, leftEncoder::getPosition, (value) -> inputs.leftPositionRotations = value); 
        ifOk(leftMotor, leftEncoder::getVelocity, (value) -> inputs.leftVelocityRPM = value);
        ifOk(
            leftMotor,
            new DoubleSupplier[] {leftMotor::getAppliedOutput, leftMotor::getBusVoltage},
            (values) -> inputs.leftAppliedVolts = values[0] * values[1]);
        ifOk(leftMotor, leftMotor::getOutputCurrent, (value) -> inputs.leftCurrentAmps = value);

        // Same thing for right motor
        ifOk(rightMotor, rightEncoder::getPosition, (value) -> inputs.rightPositionRotations = value);
        ifOk(rightMotor, rightEncoder::getVelocity, (value) -> inputs.rightVelocityRPM = value);
        ifOk(
            rightMotor,
            new DoubleSupplier[] {rightMotor::getAppliedOutput, rightMotor::getBusVoltage},
            (values) -> inputs.rightAppliedVolts = values[0] * values[1]);
        ifOk(rightMotor, rightMotor::getOutputCurrent, (value) -> inputs.rightCurrentAmps = value);

        // Update limit switch
        inputs.limitSwitch = getLimitSwitch();  
    }

    private double calculateFeedforward(int errorDirection) {
        return //ElevatorConstants.kStaticFrictionTermSpark * errorDirection + (This might not work as the friction term could be negative and then not be able to change)
        ElevatorConstants.kGravityTermSpark;
    }

    public double getElevMotorPosition() {
        return rightEncoder.getPosition();
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    @Deprecated
    public void setVoltage(double volts, SparkFlex motor) {
        motor.setVoltage(volts);
    }

    public void updateSetpoint(double setpoint) {
        this.motorSetpoint = this.clipSetpoint(setpoint);
        this.rightMotor.getClosedLoopController().setReference(this.motorSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, calculateFeedforward((int)Math.signum(this.motorSetpoint - rightEncoder.getPosition())));
    }

    public double clipSetpoint(double setpoint) {
        if(motorSetpoint > ElevatorConstants.kMaxRotations) {
            return ElevatorConstants.kMaxRotations;
        } else if(motorSetpoint < ElevatorConstants.kMinRotations) {
            return ElevatorConstants.kMinRotations;
        }
        return setpoint;
    }
}