package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

public class AlignToPose extends Command {
    private final SwerveDriveSubsystem drivetrain;
    private final Pose2d targetPose;
    private Command pathfindingCommand;
    
    public AlignToPose(SwerveDriveSubsystem drivetrain, Pose2d targetPose) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        this.pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints);
    
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        this.pathfindingCommand.schedule();
        Logger.recordOutput("Commands/AlignToPose", true);
    }

    @Override
    public boolean isFinished() {
        return this.pathfindingCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveVelocity(new ChassisSpeeds(0, 0, 0));
        Logger.recordOutput("Commands/AlignToPose", false);

        if (this.pathfindingCommand.isScheduled()) {
            this.pathfindingCommand.end(interrupted);
        }
    }
}