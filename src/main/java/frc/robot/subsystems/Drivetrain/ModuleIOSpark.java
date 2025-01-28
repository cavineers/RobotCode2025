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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.function.DoubleSupplier;
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
    private final RelativeEncoder turnEncoder;
    private final CANcoder turnAbsoluteEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Spark Configurations
    private SparkMaxConfig turnConfig;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

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
        turnEncoder = turnSpark.getEncoder();

        // Create absolute encoder
        turnAbsoluteEncoder = new CANcoder(
                switch (module) {
                    case 0 -> kFrontLeftAbsoluteEncoderPort;
                    case 1 -> kFrontRightAbsoluteEncoderPort;
                    case 2 -> kBackLeftAbsoluteEncoderPort;
                    case 3 -> kBackRightAbsoluteEncoderPort;
                    default -> 0;
                });

        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();

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
                .primaryEncoderPositionPeriodMs((int) (1000.0 / kOdometryFrequency))
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
        turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(kTurnPIDMinInput, kTurnPIDMaxInput)
                .pidf(
                        kTurnKp, 0.0,
                        kTurnKd, 0.0);
        turnConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / kOdometryFrequency))
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

        // Now apply the absolute encoder position to the turn motor relative encoder
        tryUntilOk(turnSpark,
                5,
                () -> turnEncoder.setPosition(turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI)); // Rotations of out to radians of input (AV*Out/InGearRatio*2pi)
        
        // Logger.recordOutput("ModuleAngles/Module" + module, turnAbsoluteEncoder.getPosition().getValueAsDouble()); // This can be done using the phoenix live source
        // Create odometry queues
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
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
        ifOk(
                turnSpark,
                turnEncoder::getPosition,
                (value) -> inputs.turnPosition = new Rotation2d(value));
        ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(
                turnSpark,
                new DoubleSupplier[] { turnSpark::getAppliedOutput, turnSpark::getBusVoltage },
                (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveSpark.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
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
        turnController.setReference(setpoint, ControlType.kPosition);
    }

    @Override
    public void setTurningPID(double kp, double ki, double kd) {
        turnConfig.closedLoop.pidf(kp, ki, kd, 0.0);
        tryUntilOk(
                turnSpark,
                5,
                () -> turnSpark.configure(
                        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
}