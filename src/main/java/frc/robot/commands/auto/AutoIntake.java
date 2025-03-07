package frc.robot.commands.auto;
import static frc.robot.subsystems.EndEffector.EndEffectorConstants.kEndEffectorIntakeSpeed;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndEffector;

public class AutoIntake extends Command {

    private EndEffector endEffector;

    public AutoIntake(EndEffector endEffector) {
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Logger.recordOutput("Commands/AutoIntake", true);

        endEffector.setVoltage(kEndEffectorIntakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Commands/AutoIntake", false);
        
        endEffector.setVoltage(0.0);
    }

    @Override
    public boolean isFinished() {

        return endEffector.getBumpStop();
    }
}
