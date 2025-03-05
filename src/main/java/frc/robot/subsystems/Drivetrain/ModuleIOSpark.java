package frc.robot.subsystems.Drivetrain;

import static frc.robot.subsystems.Drivetrain.SwerveDriveConstants.*;
import static frc.robot.subsystems.Drivetrain.SwerveDriveConstants.DriveConstants.*;
import static frc.robot.subsystems.Drivetrain.SwerveDriveConstants.ModuleConstants.*;
import static frc.lib.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

/**
 * Module IO implementation for Spark Max drive motor controller, Spark Max
 * turn motor controller,
 * and CanCoder absolute encoder for initial homing.
 */
public class ModuleIOSpark implements ModuleIO {
    private final double zeroRotation;

    // Hardware objects
    private final SparkBase driveSpark;
    private final SparkBase turnSpark;
    private final RelativeEncoder driveEncoder;
    private final CANcoder turnEncoder;
    private final RelativeEncoder turnRelativeEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController driveController;
    private final PIDController turnController;

    // Spark Configurations
    private SparkMaxConfig turnConfig;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    private int moduleNumber;

    public ModuleIOSpark(int module) {
        zeroRotation = switch (module) {
            case 0 -> kFrontLeftAbsoluteEncoderOffset;
            case 1 -> kFrontRightAbsoluteEncoderOffset;
            case 2 -> kBackLeftAbsoluteEncoderOffset;
            case 3 -> kBackRightAbsoluteEncoderOffset;
            default -> 0;
        };
        driveSpark = new SparkMax(
                switch (module) {
                    case 0 -> kFrontLeftDriveCanID;
                    case 1 -> kFrontRightDriveCanID;
                    case 2 -> kBackLeftDriveCanID;
                    case 3 -> kBackRightDriveCanID;
                    default -> 0;
                },
                MotorType.kBrushless);
        turnSpark = new SparkMax(
                switch (module) {
                    case 0 -> kFrontLeftTurningCanID;
                    case 1 -> kFrontRightTurningCanID;
                    case 2 -> kBackLeftTurningCanID;
                    case 3 -> kBackRightTurningCanID;
                    default -> 0;
                },
                MotorType.kBrushless);
        driveEncoder = driveSpark.getEncoder();

        // Create absolute encoder
        turnEncoder = new CANcoder(
                switch (module) {
                    case 0 -> kFrontLeftAbsoluteEncoderPort;
                    case 1 -> kFrontRightAbsoluteEncoderPort;
                    case 2 -> kBackLeftAbsoluteEncoderPort;
                    case 3 -> kBackRightAbsoluteEncoderPort;
                    default -> 0;
                });

        this.turnRelativeEncoder = this.turnSpark.getEncoder();

        driveController = driveSpark.getClosedLoopController();
        turnController = new PIDController(kTurnKp, 0, kTurnKd);
        this.turnController.enableContinuousInput(kTurnPIDMinInput, kTurnPIDMaxInput);

        // Configure drive motor
        var driveConfig = new SparkMaxConfig();
        driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(kDriveMotorCurrentLimit)
                .voltageCompensation(12.0);
        driveConfig.encoder
                .positionConversionFactor(kDriveEncoderRot2Rad)
                .velocityConversionFactor(kDriveEncoderRPM2RadPerSec)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(
                        kDriveKp, 0.0,
                        kDriveKd, 0.0);
        driveConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                driveSpark,
                5,
                () -> driveSpark.configure(driveConfig, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));

        tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

        // Configure turn motor
        turnConfig = new SparkMaxConfig();
        turnConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(kTurnMotorCurrentLimit)
                .voltageCompensation(12.0);
        turnConfig.encoder
                .positionConversionFactor(kTurningEncoderRot2Rad)
                .velocityConversionFactor(kTurningEncoderRPM2RadPerSec)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        turnConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                turnSpark,
                5,
                () -> turnSpark.configure(
                        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        this.moduleNumber = module;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        sparkStickyFault = false; // controlled by the sparkUtil class
        ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
        ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
        ifOk(
                driveSpark,
                new DoubleSupplier[] { driveSpark::getAppliedOutput, driveSpark::getBusVoltage },
                (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
        ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // Update turn inputs
        sparkStickyFault = false;
        inputs.turnPosition = this.getTurnPosition();
        inputs.turnVelocityRadPerSec = this.turnRelativeEncoder.getVelocity() / 60.0;
        ifOk(
                turnSpark,
                new DoubleSupplier[] { turnSpark::getAppliedOutput, turnSpark::getBusVoltage },
                (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        this.setTurnOpenLoop(this.turnController.calculate(this.getTurnPosition().getRadians()));
    }

    public Rotation2d getTurnPosition(){
        return Rotation2d.fromRadians(this.turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveSpark.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        output = MathUtil.clamp(output, -12.0, 12.0);
        Logger.recordOutput("SwerveStates/SetTurnVoltage" + this.moduleNumber, output);
        turnSpark.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double ffVolts = kDriveKs * Math.signum(velocityRadPerSec) + kDriveKv * velocityRadPerSec;
        driveController.setReference(
                velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint = MathUtil.inputModulus(
                rotation.getRadians(), kTurnPIDMinInput, kTurnPIDMaxInput);
        turnController.setSetpoint(setpoint);
    }

    @Override
    public void setTurningPID(double kp, double ki, double kd) {
        turnController.setPID(kp, ki, kd);
    }
}