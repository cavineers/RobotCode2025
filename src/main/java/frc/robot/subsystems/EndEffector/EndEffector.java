package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    public EndEffector(EndEffectorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() { 
        io.updateInputs(inputs); 
        Logger.processInputs("EndEffector", inputs);
    }
    
    public boolean getIR() {
        return inputs.coralPresentIR;
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void setPercentage(double percentage) {
        io.setVoltage(percentage * 12.0);
    }

    public Command setVoltageCommand(double volts) {
        return Commands.run(() -> io.setVoltage(volts), this).finallyDo(interrupted -> io.setVoltage(0));
    }

    public Command intakeCommand() {
        return Commands.run(() -> io.intake(), this).finallyDo(interrupted -> io.setVoltage(0));
    }

    public Command shootCommand() {
        return Commands.run(() -> io.shoot(), this).finallyDo(interrupted -> io.setVoltage(0));
    }
}