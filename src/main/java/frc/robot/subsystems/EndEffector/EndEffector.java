package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase{
    private final EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new  EndEffectorIOInputsAutoLogged();

    public EndEffector(EndEffectorIO io) {
        this.io = io;
}
    
    
    @Override
    public void periodic() {
        io.updateInputs(inputs); 
        Logger.processInputs("EndEffector", inputs); 
        if (inputs.leftSensor)
            setVoltage(0);
        
    }

    
    
    public Command setVoltageCommand(double volts) {
        return Commands.startEnd(() -> setVoltage(volts), () -> setVoltage(0), this);
    }

    public void setVoltage(double volts) {
        if (inputs.leftSensor){
            io.setVoltage(0);
        }else{
            io.setVoltage(volts);
        }
    }

   
   
    public void setPercentage(double percentage) {
        io.setVoltage(percentage * 12.0);
    }
}
