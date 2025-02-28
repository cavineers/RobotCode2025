package frc.robot.subsystems.Lights;

import static frc.robot.subsystems.Lights.LightsConstants.*;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorState;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Lights extends SubsystemBase {
    private Supplier<Double> elevatorPosition;
    private Supplier<ElevatorState> elevatorState;
    private Supplier<Boolean> shooterRunning;
    private Supplier<Boolean> isNearStation;

    private AddressableLED ledStrip = new AddressableLED(kPWMPort);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);


    public Lights() {
        this.ledStrip.setLength(this.ledBuffer.getLength());

        // Set the data
        this.ledStrip.setData(this.ledBuffer);
        this.ledStrip.start();

        LEDPattern red = LEDPattern.solid(Color.kRed);

        // Apply the LED pattern to the data buffer
        red.applyTo(this.ledBuffer);

        // Write the data to the LED strip
        this.ledStrip.setData(this.ledBuffer);
    }


}