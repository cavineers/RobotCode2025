package frc.robot.subsystems.Example;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import static frc.robot.subsystems.Example.ExampleConstants.*;

public class Example extends SubsystemBase {
    private final ExampleIO io;
    private final ExampleIOInputsAutoLogged inputs = new ExampleIOInputsAutoLogged();

    public Example(ExampleIO io) {
        this.io = io;
    }

    @Override
    public void periodic() { // Called once per scheduler run
        io.updateInputs(inputs); // inputs is passed as a reference here
        if (inputs.currentAmps > kCuttOffAmps) {
            io.setVoltage(0.0);
        }

        Logger.processInputs("Example", inputs); // required for logging and replay to work

        // Set the voltage to something random every periodic for fun
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public Command setVoltageCommand(double volts) {
        return Commands.startEnd(() -> this.setVoltage(volts), () -> this.setVoltage(0.0), this);
    }

    public void setPercentage(double percentage) {
        io.setVoltage(percentage * 12.0);
    }

}