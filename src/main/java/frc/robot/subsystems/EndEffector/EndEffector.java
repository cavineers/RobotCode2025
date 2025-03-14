package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    @AutoLogOutput(key="EndEffector/isShooting")
    private boolean isShooting = false;

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

    public boolean getBumpStop() {
        return inputs.coralLoadedLimit;
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void setPercentage(double percentage) {
        io.setVoltage(percentage * 12.0);
    }

    public Command setVoltageCommand(double volts) {
        return Commands.run(() -> io.setVoltage(volts), this);
    }

    public Command intakeCommand() {
        return Commands.run(() -> io.intake(), this);
    } 

    public Command shootCommand() {
        return Commands.run(() -> {
            this.isShooting = true;
            io.shoot();
        }, this).finallyDo(interrupted -> {
            this.isShooting = false;
            io.setVoltage(0);
        });
    }

    public boolean isShooting() {
        return inputs.velocityRadPerSec > 250;
    }

    public Command stopCommand() {
        return Commands.run(() -> io.setVoltage(0), this);
    }

    public boolean getIsShooting(){
        return this.isShooting;
    }
}
