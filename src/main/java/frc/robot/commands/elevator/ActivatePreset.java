package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.Elevator;

public class ActivatePreset extends Command {

    public double rotations;

    public boolean elevDone;

    private Elevator elevator;

    public ActivatePreset(Elevator elevator, double r) {
        this.addRequirements(elevator);

        this.elevator = elevator;

        rotations = r;
    }

    @Override
    public void initialize() {

        elevDone = false;
    }

    @Override
    public void execute() {

        this.elevator.io.setSetPoint(rotations);
    }

    @Override
    public void end(boolean interrupted) {
    }

    
    @Override 
    public boolean isFinished() {
        return true;
    }
}