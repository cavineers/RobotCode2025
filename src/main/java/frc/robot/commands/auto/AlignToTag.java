package frc.robot.commands.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Elastic;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.subsystems.Vision.Vision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class AlignToTag extends Command {

    private SwerveDriveSubsystem drivetrain;
    private Vision vision;
    private Translation2d offset;
    private double goalRotation;
    private int tagId;
    private Translation2d translationToPose;

    // PIDs for movement
    private PIDController translationXController = new PIDController(Constants.DriveConstants.PathPlannerDriveP*1.75, 0, 0);
    private PIDController translationYController = new PIDController(Constants.DriveConstants.PathPlannerDriveP*1.75, 0, 0);
    private PIDController rotationController = new PIDController(Constants.DriveConstants.PathPlannerTurnP, 0, 0);
    

    public AlignToTag(SwerveDriveSubsystem drivetrain, Vision vision, int tagId){
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.tagId = tagId;
        this.offset = new Translation2d(Constants.DriveConstants.kSideLength/2.0+0.1, 0); // divide by a little less than half (leaves a gap)
    }

    @Override
    public void initialize(){
        // feeding in the distance away from setpoints to PID
        this.translationXController.setSetpoint(0); // zero difference between the current and goal translation
        this.translationYController.setSetpoint(0); 
        this.rotationController.setSetpoint(0);
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute(){
        Logger.recordOutput("Commands/alignToTag", true);
        this.translationToPose = this.vision.getSingleTargetTranslation(tagId);
        this.translationToPose = translationToPose.minus(offset); // add the left, right, or center offset
        
        // Calculate chassis speeds from PID 
        double xSpeed = -this.translationXController.calculate(translationToPose.getX());
        double ySpeed = -this.translationYController.calculate(translationToPose.getY());
        double turnSpeed = this.rotationController.calculate(drivetrain.getRotation().getRadians()); // ambiguity can cause rotation to be off --> use gyro!!

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        Logger.recordOutput("AlignToTarget/Speeds", speeds);
        Logger.recordOutput("AlignToTarget/TranslationToPose", translationToPose);
        Logger.recordOutput("AlignToTarget/Rotation", drivetrain.getRotation().getRadians());

        this.drivetrain.driveVelocity(speeds);


        double currentDist = this.vision.getFieldTagPose(tagId).getTranslation().toTranslation2d().getDistance(this.drivetrain.getPose().getTranslation());
        Logger.recordOutput("AlignToTarget/DistanceBetweenPoses", currentDist);
    }

    @Override
    public boolean isFinished(){
        return 
            this.translationXController.atSetpoint() &&
            this.translationYController.atSetpoint() &&
            this.rotationController.atSetpoint() || 
            !this.vision.hasTag(this.tagId); // this could cause early cancellations -- needs real testing


    }

    @Override
    public void end(boolean interrupted){
        this.drivetrain.driveVelocity(new ChassisSpeeds()); // stop drivetrain
        Logger.recordOutput("Commands/alignToTag", false);

        if (!this.vision.hasTag(this.tagId)){
            Elastic.Notification notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.WARNING, "AlignToTag", "Tag ID: {" + this.tagId + "} not found");
            Elastic.sendNotification(notification);
        }
    }

    
}
