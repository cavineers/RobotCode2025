package frc.robot.commands.auto;

import static frc.robot.subsystems.Elevator.ElevatorConstants.kSetPointTolerance;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;

public class AutoElevatorPreset extends Command {

    private Elevator elevator;
    private Command goToPreset;
    private double setpointGoal;

    public AutoElevatorPreset(Elevator elevator, double setPoint) {
        setpointGoal = setPoint;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        this.goToPreset = elevator.goToPresetCommand(setpointGoal);
        this.goToPreset.schedule();
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

        return elevator.IsAtSetpoint();
    }
}
