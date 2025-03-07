package frc.robot.commands.auto;
import static frc.robot.subsystems.EndEffector.EndEffectorConstants.kEndEffectorIntakeSpeed;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CanRangeArray.CanRangeArray;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.subsystems.Drivetrain.SwerveDriveConstants.DriveConstants;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;

public class AutoShoot extends Command {

    private EndEffector endEffector;

    public AutoShoot(EndEffector endEffector) {
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Logger.recordOutput("Commands/AutoShoot", true);

         endEffector.setVoltage(kEndEffectorIntakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Commands/AutoShoot", false);
        endEffector.setVoltage(0.0);
    }

    @Override
    public boolean isFinished() {

        return endEffector.getIR();
    }
}
