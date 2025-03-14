package frc.robot.commands.auto;
import static frc.robot.subsystems.EndEffector.EndEffectorConstants.kEndEffectorShootSpeed;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndEffector;

public class AutoShoot extends Command {

    private EndEffector endEffector;

    private Debouncer endDebouncer;

    public AutoShoot(EndEffector endEffector) {
        addRequirements(endEffector);
        this.endEffector = endEffector;
        this.endDebouncer = new Debouncer(0.25);
    }

    @Override
    public void initialize() {
        System.out.println("STARTED");
    }

    @Override
    public void execute() {
        Logger.recordOutput("Commands/AutoShoot", true);

        endEffector.setPercentage(kEndEffectorShootSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Commands/AutoShoot", false);
        endEffector.setVoltage(0.0);
        System.out.println("FINISHED");
    }

    @Override
    public boolean isFinished() {

        return this.endDebouncer.calculate(!endEffector.getIR());
    }
}
