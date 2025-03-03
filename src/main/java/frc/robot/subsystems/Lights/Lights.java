package frc.robot.subsystems.Lights;

import static frc.robot.subsystems.Lights.LightsConstants.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorState;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import edu.wpi.first.units.Units;
import static frc.robot.subsystems.Lights.LightsConstants.*;


import org.littletonrobotics.junction.Logger;

public class Lights extends SubsystemBase {
    private Supplier<Double> elevatorVelocity;
    private Supplier<ElevatorState> elevatorState;
    private Supplier<Boolean> shooterRunning;
    private Supplier<Boolean> isNearStation;

    private AddressableLED ledStrip = new AddressableLED(kPWMPort);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(150);

    // Four sections of the LED strip

    private AddressableLEDBufferView createLEDView(int start, int count) {
        return new AddressableLEDBufferView(ledBuffer, start, start + count - 1);
    }
    
    private AddressableLEDBufferView funnelLeft = createLEDView(0, kFunnelCount);
    private AddressableLEDBufferView topLeft = createLEDView(kFunnelCount, kTopCount);
    private AddressableLEDBufferView elevatorLeft = createLEDView(kFunnelCount + kTopCount, kElevatorCount);
    
    private AddressableLEDBufferView funnelRight = createLEDView(kRightSideStart, kFunnelCount);
    private AddressableLEDBufferView topRight = createLEDView(kRightSideStart + kFunnelCount, kTopCount);
    private AddressableLEDBufferView elevatorRight = createLEDView(kRightSideStart + kFunnelCount + kTopCount, kElevatorCount);

    // Alliance Station 
    private Optional<Alliance> ally = DriverStation.getAlliance();

    // Our LED strip has a density of 120 LEDs per meter
    public Lights(Supplier<Double> elevatorVelocity, Supplier<Boolean> shooterRunning) {
        this.ledStrip.setLength(this.ledBuffer.getLength());

        // Set the data
        this.ledStrip.setData(this.ledBuffer);
        this.ledStrip.start();
        // Assign each section a color for tuning
        LEDPattern pattern = LEDPattern.solid(Color.kWhite).breathe(Units.Seconds.of(1));
        LEDPattern pattern2 = LEDPattern.solid(Color.kRed).breathe(Units.Seconds.of(1));

        pattern.applyTo(funnelLeft);
        pattern2.applyTo(funnelRight);
        pattern.applyTo(topLeft);
        pattern2.applyTo(topRight);
        pattern.applyTo(elevatorLeft);
        pattern2.applyTo(elevatorRight);

        this.ledStrip.setData(this.ledBuffer);

        this.elevatorVelocity = elevatorVelocity;
        this.shooterRunning = shooterRunning;
        System.out.println("LIGHTS ACTIVATED");
    }

    @Override
    public void periodic(){
        double velocity = elevatorVelocity.get();
        boolean shooter = shooterRunning.get();

        // Go in order of importance (Last one will overwrite the previous)

        if (Math.abs(this.elevatorVelocity.get()) > LightsConstants.kElevatorVelocityThreshold) {
            this.setElevatorEffect(velocity);
        }
        if (shooter) {
            this.setShooterEffect();
        }


        // Must update the LED strip with the new data
        this.ledStrip.setData(this.ledBuffer);
    }

    private Color getAllianceStationColor() {
        if (ally.isPresent() && ally.get() == Alliance.Blue) {
            return Color.kBlue;
        } else {
            return Color.kRed;
        }
    }

    private void setElevatorEffect(double velocity){
        Color color = this.getAllianceStationColor();
        LEDPattern pattern = LEDPattern.steps(Map.of(0, color, 0.25, Color.kBlack));

        // Calculate the speed of the pattern based on the velocity of the elevator
        double normalizedVelocity = velocity / 2000.0; // rando value

        pattern = pattern.scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(normalizedVelocity));

        pattern.applyTo(elevatorLeft);
        pattern.applyTo(elevatorRight);
        pattern.applyTo(topLeft);
        pattern.applyTo(topRight);
        pattern.applyTo(funnelLeft);
        pattern.applyTo(funnelRight);

    }

    private void setShooterEffect(){
        Color color = Color.kYellow;
        LEDPattern pattern = LEDPattern.rainbow(255, 175);
        pattern = pattern.scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(1), kLedSpacing);
        pattern.applyTo(topLeft);
        pattern.applyTo(topRight);
        pattern.applyTo(funnelLeft);
        pattern.applyTo(funnelRight);
        pattern.applyTo(elevatorLeft);
        pattern.applyTo(elevatorRight);
    }
}