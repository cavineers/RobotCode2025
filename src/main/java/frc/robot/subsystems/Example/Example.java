package frc.robot.subsystems.Example;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Example extends SubsystemBase {
    private final ExampleIO io;
    private final ExampleIOInputsAutoLogged inputs = new ExampleIOInputsAutoLogged();

    public Example(ExampleIO io) {
        this.io = io;
    }

    @Override
    public void periodic() { // Called once per scheduler run
        io.updateInputs(inputs); // inputs is passed as a reference here
        Logger.processInputs("Example", inputs); // required for logging and replay to work

        // Set the voltage to something random every periodic for fun
        io.setVoltage(Math.random() * 12.0);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void setPercentage(double percentage) {
        io.setVoltage(percentage * 12.0);
    }
}