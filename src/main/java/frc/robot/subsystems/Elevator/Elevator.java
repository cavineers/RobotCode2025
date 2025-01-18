package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io) {

        this.io = io;
    }

    @Override
    public void periodic() {

        io.updateinputs(inputs);
        Logger.processInputs("Elevator", inputs);

        io.setVoltage(Math.random() * 12);
    }

    public void setVoltage(double volts) {

        io.setVoltage(volts);
    }

    public void setPercentage(double percentage) {

        io.setVoltage(percentage * 12);
    }
}