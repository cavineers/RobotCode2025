package frc.robot.commands.auto;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.subsystems.Drivetrain.SwerveDriveConstants.DriveConstants;

public class AlignToClosestTag extends Command {

    private SwerveDriveSubsystem drivetrain;

    // PIDs for movement
    private PIDController translationXController = new PIDController(DriveConstants.PathPlannerDriveP, 0, 0.0);
    private PIDController translationYController = new PIDController(DriveConstants.PathPlannerDriveP, 0, 0.0);
    private PIDController rotationController = new PIDController(DriveConstants.PathPlannerTurnP+1, 0, 0);

    private Supplier<Pose2d> targetPoseSupplier;
    private boolean speedLow = false;

    public AlignToClosestTag(SwerveDriveSubsystem drivetrain, Supplier<Pose2d> targetPoseSupplier){
        this.drivetrain = drivetrain;
        this.rotationController.setTolerance(0.01);
        this.translationXController.setTolerance(0.02);
        this.translationYController.setTolerance(0.02);
        this.targetPoseSupplier = targetPoseSupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        // feeding in the distance away from setpoints to PID
        this.translationXController.setSetpoint(0); // zero difference between the current and goal translation
        this.translationYController.setSetpoint(0); 
        this.rotationController.setSetpoint(this.targetPoseSupplier.get().getRotation().getRadians()); // set the rotation setpoint to the goal rotation
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
        this.speedLow = false;
    }

    @Override
    public void execute(){
        Logger.recordOutput("Commands/alignToTag", true);

        Pose2d drivePose = this.drivetrain.getPose();

        Translation2d currentTranslation = new Translation2d(this.targetPoseSupplier.get().getX() - drivePose.getX(), this.targetPoseSupplier.get().getY() - drivePose.getY());
        // Calculate chassis speeds from PID 
        double xSpeed = -this.translationXController.calculate(currentTranslation.getX());
        double ySpeed = -this.translationYController.calculate(currentTranslation.getY());
        double turnSpeed = this.rotationController.calculate(drivePose.getRotation().getRadians()); // ambiguity can cause rotation to be off --> use gyro!!

       

        // boolean flipped = this.drivetrain.shouldFlipPose();

        // Convert to field relative chassis speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, drivetrain.getPose().getRotation());
            // flipped ? drivePose.getRotation().plus(Rotation2d.fromRadians(Math.PI)) : drivetrain.getPose().getRotation());
        
        Logger.recordOutput("AlignToTarget/Speeds", speeds);
        Logger.recordOutput("AlignToTarget/TargetPose", this.targetPoseSupplier.get());
        Logger.recordOutput("AlignToTarget/Translation2D", currentTranslation);
        Logger.recordOutput("Commands/alignToTag", true);
        if (DriverStation.isAutonomous()){
            PPHolonomicDriveController.overrideXFeedback(() -> {
                return speeds.vxMetersPerSecond;
            });
            PPHolonomicDriveController.overrideYFeedback(() -> {
                return speeds.vyMetersPerSecond;
            });
            PPHolonomicDriveController.overrideRotationFeedback(() -> {
                return speeds.omegaRadiansPerSecond;
            });
        }
        this.speedLow = determineSpeedThreshold(speeds);
        // if (!this.speedLow){
        this.drivetrain.driveVelocity(speeds);
        // }
        
        

    }

    private boolean determineSpeedThreshold(ChassisSpeeds speeds){
        return false;
    }
    @Override
    public boolean isFinished(){
        return 
            this.translationXController.atSetpoint() &&
            this.translationYController.atSetpoint() &&
            this.rotationController.atSetpoint() ||
            this.speedLow;
    }

    @Override
    public void end(boolean interrupted){
        this.drivetrain.driveVelocity(new ChassisSpeeds()); // stop drivetrain
        Logger.recordOutput("Commands/alignToTag", false);
        PPHolonomicDriveController.clearFeedbackOverrides();   
    }

    
}
