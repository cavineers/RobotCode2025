package frc.robot.commands.auto;

import static frc.robot.subsystems.Elevator.ElevatorConstants.kSetPointTolerance;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;

public class AutoElevatorPreset extends Command {

    private Elevator elevator;
    private double setpointGoal;

    public AutoElevatorPreset(Elevator elevator, double setPoint) {
        setpointGoal = setPoint;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        this.elevator.io.setClosedLoop(true);
        this.elevator.io.updateSetpoint(this.setpointGoal);
        Logger.recordOutput("Commands/AutoElevatorPreset", true);

    }

    @Override
    public void execute() {
        Logger.recordOutput("Commands/AutoElevatorPreset", true);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Commands/AutoElevatorPreset", false);
    }

    @Override
    public boolean isFinished() {

        return Math.abs(elevator.getElevatorPosition() - this.setpointGoal) < kSetPointTolerance;
    }
}
