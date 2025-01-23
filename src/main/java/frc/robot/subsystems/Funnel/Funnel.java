package frc.robot.subsystems.Funnel;

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
            Logger.processInput("INPUT EXAMPLE", inputs);
        //What inputs do we want here ^
    
        io.setVoltage(Math.random() * 12.0); 
        //Power level here ^
    }

public void setVoltage(double volts) {
    io.setVoltage (volts);
    }

    public void setPercentage(double percentage) {
        io.setVoltage(percentage * 12.0);
        //Something important is here ^
    }

}