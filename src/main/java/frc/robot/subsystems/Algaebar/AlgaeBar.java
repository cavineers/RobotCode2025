package frc.robot.subsystems.Algaebar;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Algaebar.AlgaeBarIO.AlgaeBarIOInputs;

import org.littletonrobotics.junction.Logger;

public class AlgaeBar extends SubsystemBase {
    private final AlgaeBarIO io;
    private final AlgaeBarIOInputsAutoLogged inputs = new AlgaeBarIOInputsAutoLogged();

    public AlgaeBar(AlgaeBarIO io) {
        this.io = io;

    }

    @Override
    public void periodic() {//called once per scheduler run
        io.updateInputs(inputs); //inputs is passed as a reference here
        Logger.processInputs("AlgaeBar", inputs); //required for logging and replay to work

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
}
