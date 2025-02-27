package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import static frc.robot.subsystems.Drivetrain.SwerveDriveConstants.*;

public class DriveToPose extends Command {
    private Command pathfindingCommand;
    private final SwerveDriveSubsystem drivetrain;
    private final Supplier<Pose2d> targetPoseSupplier;
    private Pose2d targetPose;
    private final PathConstraints constraints;
    
    public DriveToPose(SwerveDriveSubsystem drivetrain, Supplier<Pose2d> targetPoseSupplier) {
        // Create the constraints to use while pathfinding
        this.constraints = new PathConstraints(
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond, DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
            DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond, DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.drivetrain = drivetrain;
        this.targetPoseSupplier = targetPoseSupplier;
        addRequirements(drivetrain);
    }

    public DriveToPose(SwerveDriveSubsystem drivetrain, Pose2d targetPose){
        // Handles the case where we just want to create drive to a single constant pose commands
        this.constraints = new PathConstraints(
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond, DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
            DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond, DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;

        this.targetPoseSupplier = () -> {return targetPose;};
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        this.targetPose = targetPoseSupplier.get();
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        this.pathfindingCommand = drivetrain.shouldFlipPose()
            ? AutoBuilder.pathfindToPoseFlipped(targetPose, constraints) 
            : AutoBuilder.pathfindToPose(targetPose, constraints);
        this.pathfindingCommand.schedule();
    }

    @Override 
    public void execute() {
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
        this.pathfindingCommand.cancel();
        if (interrupted) {
            System.out.println("driveToPose was interrupted");
        }
        Logger.recordOutput("Commands/driveToPose", false);
    }
}