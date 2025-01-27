package frc.robot.subsystems.Funnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase{
    private final FunnelIO io;
    private final FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();
    
    public Funnel(FunnelIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Funnel", inputs);
        
    }

    public void setVoltage(double volts) {
        io.setVoltage (volts);
    }

    public void setPercentage(double percentage) {
        io.setVoltage(percentage * 12.0);
        
    }

    public Command setVoltageCommand(double volts) {
        return Commands.run(() -> io.setVoltage(volts), this).finallyDo(interrupted -> io.setVoltage(0));
    }

}