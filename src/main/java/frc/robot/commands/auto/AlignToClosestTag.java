package frc.robot.commands.auto;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Elastic;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.subsystems.Vision.Vision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.subsystems.Drivetrain.SwerveDriveConstants.DriveConstants;

public class AlignToClosestTag extends Command {

    private SwerveDriveSubsystem drivetrain;
    private Vision vision;
    private Translation2d offset;
    private double goalRotation;
    private int tagId;

    // PIDs for movement
    private PIDController translationXController = new PIDController(DriveConstants.PathPlannerDriveP, 0, 0);
    private PIDController translationYController = new PIDController(DriveConstants.PathPlannerDriveP, 0, 0);
    private PIDController rotationController = new PIDController(DriveConstants.PathPlannerTurnP, 0, 0);

    private Supplier<Pose2d> targetPoseSupplier;

    public AlignToClosestTag(SwerveDriveSubsystem drivetrain, Supplier<Pose2d> targetPoseSupplier){
        this.drivetrain = drivetrain;
        this.rotationController.setTolerance(0.01);
        this.translationXController.setTolerance(0.01);
        this.translationYController.setTolerance(0.01);
        this.targetPoseSupplier = targetPoseSupplier;
    }

    @Override
    public void initialize(){
        // feeding in the distance away from setpoints to PID
        this.translationXController.setSetpoint(this.targetPoseSupplier.get().getX()); // zero difference between the current and goal translation
        this.translationYController.setSetpoint(this.targetPoseSupplier.get().getY()); 
        this.rotationController.setSetpoint(this.targetPoseSupplier.get().getRotation().getRadians()); // set the rotation setpoint to the goal rotation
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute(){
        Logger.recordOutput("Commands/alignToTag", true);

        Pose2d drivePose = this.drivetrain.getPose();
        // Calculate chassis speeds from PID 
        double xSpeed = this.translationXController.calculate(drivePose.getX());
        double ySpeed = this.translationYController.calculate(drivePose.getY());
        double turnSpeed = this.rotationController.calculate(drivePose.getRotation().getRadians()); // ambiguity can cause rotation to be off --> use gyro!!

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        Logger.recordOutput("AlignToTarget/Speeds", speeds);

        this.drivetrain.driveVelocity(speeds);
    }

    @Override
    public boolean isFinished(){
        return 
            this.translationXController.atSetpoint() &&
            this.translationYController.atSetpoint() &&
            this.rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted){
        this.drivetrain.driveVelocity(new ChassisSpeeds()); // stop drivetrain
        Logger.recordOutput("Commands/alignToTag", false);
    }

    
}
