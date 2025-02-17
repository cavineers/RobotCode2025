package frc.robot.subsystems.Elevator;

import static frc.lib.SparkUtil.*;

import static frc.robot.subsystems.Elevator.ElevatorConstants.kLeftMotorCanID;
import static frc.robot.subsystems.Elevator.ElevatorConstants.kRightMotorCanID;
import static frc.robot.subsystems.Elevator.ElevatorConstants.kTolerance;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOSpark implements ElevatorIO {
    
    private final SparkMax rightMotor = new SparkMax(kLeftMotorCanID, MotorType.kBrushless);
    private final SparkMax leftMotor = new SparkMax(kRightMotorCanID, MotorType.kBrushless);

    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

    private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchID);

    private LoggedNetworkNumber tuningP = new LoggedNetworkNumber("/Tuning/Elevator/P", ElevatorConstants.kProportionalGainSpark);
    private LoggedNetworkNumber tuningD = new LoggedNetworkNumber("/Tuning/Elevator/D", ElevatorConstants.kDerivativeTermSpark);
    private LoggedNetworkNumber tuningG = new LoggedNetworkNumber("/Tuning/Elevator/G", ElevatorConstants.kGravityTermSpark); 

    @AutoLogOutput(key = "Elevator/Setpoint")
    private double motorSetpoint = 0;

    private SparkMaxConfig config;

    public ElevatorIOSpark() {
        // Set up the PID controller on Spark Max
        config = new SparkMaxConfig();
        config
            .inverted(true)
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

        var leftMotorConfig = new SparkMaxConfig().apply(config);
        leftMotorConfig.follow(rightMotor, true);
        tryUntilOk(
            leftMotor,
            5,
            () -> leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Update for right motor
        ifOk(rightMotor, rightEncoder::getPosition, (value) -> inputs.rightPositionRotations = value);
        ifOk(rightMotor, rightEncoder::getVelocity, (value) -> inputs.rightVelocityRPM = value);
        ifOk(
            rightMotor,
            new DoubleSupplier[] {rightMotor::getAppliedOutput, rightMotor::getBusVoltage},
            (values) -> inputs.rightAppliedVolts = values[0] * values[1]);
        ifOk(rightMotor, rightMotor::getOutputCurrent, (value) -> inputs.rightCurrentAmps = value);

        // Update limit switch
        inputs.limitSwitch = getLimitSwitch(); 

        // Update PID Loop if necessary
        if (ElevatorConstants.kTuningMode) {
            updatePID();
        }
        
    }

    private double calculateFeedforward(int errorDirection) {
        return this.tuningG.get();
    }

    public double getElevMotorPosition() {
        return rightEncoder.getPosition();
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    @Deprecated
    public void setVoltage(double volts, SparkMax motor) {
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

    private void updatePID(){
        double currentP = rightMotor.configAccessor.closedLoop.getP();
        double currentD = rightMotor.configAccessor.closedLoop.getD();

        if (tuningP.get() != currentP || tuningD.get() != currentD) {
            config.closedLoop.pidf(tuningP.get(), 0, tuningD.get(), 0);
            tryUntilOk(
                rightMotor,
                5,
                () -> rightMotor.configure(config, ResetMode.kNoResetSafeParameters,
                        PersistMode.kPersistParameters));
        }
    }
}