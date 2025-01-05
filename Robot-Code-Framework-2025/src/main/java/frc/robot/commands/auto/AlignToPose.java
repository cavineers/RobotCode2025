package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.Logger;

public class AlignToPose extends Command {
    private final SwerveDriveSubsystem drivetrain;
    private final Pose2d targetPose;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    
    public AlignToPose(SwerveDriveSubsystem drivetrain, Pose2d targetPose) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        
        xController = new PIDController(0.5, 0, 0);
        yController = new PIDController(0.5, 0, 0);
        rotationController = new PIDController(0.5, 0, 0);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        rotationController.reset();
        Logger.recordOutput("Commands/AlignToPose", true);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();
        
        double xSpeed = xController.calculate(
            currentPose.getX(), 
            targetPose.getX()
        );
        
        double ySpeed = yController.calculate(
            currentPose.getY(), 
            targetPose.getY()
        );
        
        double rotationSpeed = rotationController.calculate(
            currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians()
        );

        drivetrain.driveVelocity(new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(xController.getPositionError()) < 0.05 &&
               Math.abs(yController.getPositionError()) < 0.05 &&
               Math.abs(rotationController.getPositionError()) < 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveVelocity(new ChassisSpeeds(0, 0, 0));
        Logger.recordOutput("Commands/AlignToPose", false);
    }
}