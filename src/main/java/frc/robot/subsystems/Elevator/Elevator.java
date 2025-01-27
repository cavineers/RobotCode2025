package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    public final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io) {
        this.io = io;

    }

    @Override
    public void periodic() { 
        io.updateInputs(inputs); 
        Logger.processInputs("Elevator", inputs);

        io.updateSetPoint();
        io.checkBoundry();
    }

    public Command setVoltageCommand(double volts) {
        return Commands.run(() -> io.setVoltage(volts), this).finallyDo(interrupted -> io.setVoltage(0));
    }

    public Command goToPresetCommand(double rotations) {
        return Commands.run(() -> io.setSetPoint(rotations), this).finallyDo(interrupted -> io.setVoltage(0));
    }
}