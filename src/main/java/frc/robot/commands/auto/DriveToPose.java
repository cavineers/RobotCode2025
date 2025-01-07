package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

public class DriveToPose extends Command {
    private Command pathfindingCommand;
    
    public DriveToPose(SwerveDriveSubsystem drivetrain, Pose2d targetPose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        this.pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    @Override
    public void initialize() {
        this.pathfindingCommand.schedule();
        Logger.recordOutput("Commands/driveToPose", true);
    }

    @Override
    public boolean isFinished() {
        if (pathfindingCommand.isScheduled()) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        if (this.pathfindingCommand.isScheduled()) {
            this.pathfindingCommand.cancel();
        }
        if (interrupted) {
            System.out.println("driveToPose was interrupted");
        }
        Logger.recordOutput("Commands/driveToPose", false);
    }
}