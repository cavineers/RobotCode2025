package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorConstants;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    public final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Supplier<Boolean> needsSafety;

    public Elevator(ElevatorIO io, Supplier<Boolean> needsSafety) {
        this.io = io;
        this.needsSafety = needsSafety;
    }

    @Override
    public void periodic() { 
        io.updateInputs(inputs); 
        io.checkBoundry();
        io.updateSetpoint();

        Logger.processInputs("Elevator", inputs);

        if (needsSafety.get()) {
            //io.setSetpoint()
            System.out.println("Elevator needs safety");
            
        }
    }

    public Command setVoltageCommand(double volts) {
        return Commands.run(() -> io.setVoltage(volts), this).finallyDo(interrupted -> io.setVoltage(0));
    }

    public Command goToPresetCommand(double rotations) {
        return Commands.run(() -> io.setSetpoint(rotations), this).finallyDo(interrupted -> io.setVoltage(0));
    }
}