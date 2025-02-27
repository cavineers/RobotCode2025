package frc.robot.subsystems.AlgaeBar;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Algaebar.AlgaeBarIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class AlgaeBar extends SubsystemBase {
    private final AlgaeBarIO io;
    private final AlgaeBarIOInputsAutoLogged inputs = new AlgaeBarIOInputsAutoLogged();

    public AlgaeBar(AlgaeBarIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeBar", inputs);
    }

    public Command setDeployVoltageCommand(double volts) {
        return Commands.run(() -> io.setDeployVoltage(volts), this).finallyDo(interrupted -> io.setDeployVoltage(0));
    }

    public Command setIntakeVoltageCommand(double volts) {
        return Commands.run(() -> io.setIntakeVoltage(volts), this).finallyDo(interrupted -> io.setIntakeVoltage(0));
    }

    public Command deployCommand() {
        return Commands.run(() -> io.deploy(), this).finallyDo(interrupted -> io.setIntakeVoltage(0));
    }

    public Command intakeCommand() {
        return Commands.run(() -> io.retract(), this).finallyDo(interrupted -> io.setIntakeVoltage(0));
    }

    public void setDeployVoltage(double volts) {
        io.setDeployVoltage(volts);
    }

    public void setIntakeVoltage(double volts) {
        io.setIntakeVoltage(volts);
    }
}