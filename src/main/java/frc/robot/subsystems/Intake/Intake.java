package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new  IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
}
    @Override
    public void periodic() {
        io.updateInputs(inputs); 
        Logger.processInputs("Intake", inputs); 

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
