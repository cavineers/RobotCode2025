package frc.robot.subsystems.ExampleKraken;

import static frc.robot.subsystems.ExampleKraken.ExampleKrakenConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class ExampleKrakenIOKraken implements ExampleKrakenIO {

    private final TalonFX motor;
    private final VelocityVoltage velocityControl;
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    
    // WPILib Alerts for error handling
    private final Alert climberConfigAlert = new Alert("Climber motor config failed", AlertType.kError);
    private final Alert setPIDAlert = new Alert("Climber setPID failed", AlertType.kWarning);

    public ExampleKrakenIOKraken() {
        motor = new TalonFX(kExampleCanID);
        
        velocityControl = new VelocityVoltage(0)
            .withSlot(0)
            .withEnableFOC(kEnableFOC);

        var climberConfig = new TalonFXConfiguration();
        
        // Leader motor setup
        climberConfig.MotorOutput.NeutralMode = kNeutralMode;
        climberConfig.MotorOutput.Inverted = kMotorInverted;
        climberConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        climberConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
        climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply configurations with error checking
        StatusCode climberStatus = motor.getConfigurator().apply(climberConfig);
        
        climberConfigAlert.set(!climberStatus.isOK());
        
        // Configure signal update frequencies for real-time control
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, // 100Hz for velocity control
            motor.getVelocity()
        );
        
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, // 50Hz for telemetry
            motor.getMotorVoltage(),
            motor.getSupplyCurrent(),
            motor.getDeviceTemp()
        );
        
        // Optimize CAN bus utilization by reducing unused signals
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ExampleKrakenIOInputsAutoLogged inputs) {
        inputs.velocityRotationsPerSec = motor.getVelocity().getValueAsDouble();
        inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.positionRotations = motor.getPosition().getValueAsDouble();
        
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageControl.withOutput(volts));
    }
}