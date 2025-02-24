package frc.robot.subsystems.Algaebar;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaeBar extends SubsystemBase {
    private final AlgaeBarIO io;
    private final AlgaeBarIOInputsAutoLogged inputs = new AlgaeBarIOInputsAutoLogged();

    public AlgaeBar(AlgaeBarIO io) {
        this.io = io;
        io.initializeDutyEncoder();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs); 
        Logger.processInputs("AlgaeBar", inputs); 
    }

    public Command setPivotVoltageCommand(double volts) {
        return Commands.run(() -> io.setPivotVoltage(volts), this).finallyDo(interrupted -> io.setPivotVoltage(0));
    }

    public Command setAlgaeBarVoltageCommand(double volts) {
        return Commands.run(() -> io.setAlgaeBarVoltage(volts), this).finallyDo(interrupted -> io.setAlgaeBarVoltage(0));
    }

    public void setPivotVoltage(double volts) {
        io.setPivotVoltage(volts);
    }

    public void setAlgaeBarVoltage(double volts){
        io.setAlgaeBarVoltage(volts);
    }

    public void setPivotPercentage(double percentage) {
        io.setPivotVoltage(percentage * 12.0);
    }
    
    public void setAlgaeBarPercentage(double percentage) {
        io.setAlgaeBarVoltage(percentage * 12.0);
    }
}