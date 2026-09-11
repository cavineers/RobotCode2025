package frc.robot.subsystems.ExampleKraken;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ExampleKrakenConstants {
    public static final int kExampleCanID = 1;

    public static final boolean kEnableFOC = false;
    // Motor configuration Kraken X60
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue kMotorInverted = InvertedValue.CounterClockwise_Positive;

    public static final double kSupplyCurrentLimit = 20.0; // Amps
    public static final double kStatorCurrentLimit = 40.0; // Amps

    // Add constants here
}
