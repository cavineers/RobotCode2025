package frc.robot.subsystems.Lights;

import static frc.robot.subsystems.Lights.LightsConstants.kLedSpacing;
import static frc.robot.subsystems.Lights.LightsConstants.kPWMPort;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    private Supplier<Double> elevatorVelocity;
    private Supplier<Boolean> shooterRunning;
    private Supplier<Boolean> bumpStop;

    private AddressableLED ledStrip = new AddressableLED(kPWMPort);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(111);




    // Four sections of the LED strip
    private Timer intakeTimer = new Timer();
    
    private AddressableLEDBufferView elevatorLeft = new AddressableLEDBufferView(ledBuffer, 0, 19);
    private AddressableLEDBufferView topLeft = new AddressableLEDBufferView(ledBuffer, 20, 33);
    private AddressableLEDBufferView funnelLeft = new AddressableLEDBufferView(ledBuffer, 34, 54);
    
    private AddressableLEDBufferView funnelRight = new AddressableLEDBufferView(ledBuffer, 55, 75);
    private AddressableLEDBufferView topRight = new AddressableLEDBufferView(ledBuffer, 76, 89);
    private AddressableLEDBufferView elevatorRight = new AddressableLEDBufferView(ledBuffer, 90, 110);

    // Alliance Station 
    private Optional<Alliance> ally = DriverStation.getAlliance();

    // Our LED strip has a density of 120 LEDs per meter
    public Lights(Supplier<Double> elevatorVelocity, 
        Supplier<Boolean> shooterRunning, 
        Supplier<Boolean> isNearStation, 
        Supplier<Boolean> bumpStop,
        Supplier<Boolean> intakeInPosition
        ) {
        this.ledStrip.setLength(this.ledBuffer.getLength());
        this.shooterRunning = shooterRunning;
        this.bumpStop = bumpStop;
        this.elevatorVelocity = elevatorVelocity;
        // Set the data
        this.ledStrip.setData(this.ledBuffer);
        this.ledStrip.start();
        // Assign each section a color for tuning

        this.elevatorVelocity = elevatorVelocity;
        this.shooterRunning = shooterRunning;
        this.intakeTimer.start();
    }

    @Override
    public void periodic(){

        LEDPattern basePattern = LEDPattern.solid(Color.kDarkRed);
        LEDPattern breathePattern = basePattern.breathe(Units.Seconds.of(5));

        breathePattern.applyTo(topLeft);
        breathePattern.applyTo(topRight);
        breathePattern.applyTo(funnelLeft);
        breathePattern.applyTo(funnelRight);
        breathePattern.applyTo(elevatorLeft);
        breathePattern.applyTo(elevatorRight);
        // Go in order of importance (Last one will overwrite the previous)

        if (this.bumpStop.get()){
            LEDPattern intakePattern = LEDPattern.solid(Color.kGreen);
            intakePattern.applyTo(topLeft);
            intakePattern.applyTo(topRight);
            intakePattern.applyTo(funnelLeft);
            intakePattern.applyTo(funnelRight);
            intakePattern.applyTo(elevatorLeft);
            intakePattern.applyTo(elevatorRight);
        }

        if (Math.abs(elevatorVelocity.get()) > LightsConstants.kElevatorVelocityThreshold) {
            this.setElevatorEffect(elevatorVelocity.get());
        }
        if (shooterRunning.get()){
            this.setShooterEffect();
        }
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
        double normalizedVelocity = velocity / 10000.0; // rando value

        pattern = pattern.scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(normalizedVelocity), kLedSpacing);
        pattern.applyTo(elevatorLeft);
        pattern.applyTo(elevatorRight);
        pattern.applyTo(topLeft);
        pattern.applyTo(topRight);
        pattern.applyTo(funnelLeft);
        pattern.applyTo(funnelRight);
    }

    private void setShooterEffect(){
        LEDPattern pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kDarkRed, Color.kBlue);
        pattern = pattern.scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(0.5), kLedSpacing);
        pattern = pattern.atBrightness(Units.Percent.of(100));
        pattern.applyTo(topLeft);
        pattern.applyTo(topRight);
        pattern.applyTo(funnelLeft);
        pattern.applyTo(funnelRight);
        pattern.applyTo(elevatorLeft);
        pattern.applyTo(elevatorRight);
    }
}