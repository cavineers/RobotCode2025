package frc.robot.subsystems.Dealgaefier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Dealgaefier extends SubsystemBase {
    private final DealgaefierIO io;
    private final DealgaefierIOInputsAutoLogged inputs = new DealgaefierIOInputsAutoLogged();

    public Dealgaefier(DealgaefierIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Dealgaefier", inputs);

        // Set the voltage to something random every periodic for fun
        io.setVoltage(Math.random() * 12.0);
    }

    public Command setVoltageCommand(double volts) {
        return Commands.run(() -> io.setVoltage(volts), this).finallyDo(interrupted -> io.setVoltage(0));
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void setPercentage(double percentage) {
        io.setVoltage(percentage * 12.0);
    }

    public Command pivotCommand() {
        return Commands.run(() -> io.pivot(), this).finallyDo(interrupted -> io.setVoltage(0));
    }

}