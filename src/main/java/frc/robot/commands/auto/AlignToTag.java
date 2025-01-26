package frc.robot.commands.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
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
    private PIDController translationXController = new PIDController(Constants.DriveConstants.PathPlannerDriveP, 0, 0);
    private PIDController translationYController = new PIDController(Constants.DriveConstants.PathPlannerDriveP, 0, 0);
    private PIDController rotationController = new PIDController(Constants.DriveConstants.PathPlannerTurnP, 0, 0);
    

    public AlignToTag(SwerveDriveSubsystem drivetrain, Vision vision, int tagId){
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.tagId = tagId;
        this.offset = new Translation2d(Constants.DriveConstants.kSideLength, 0); // 35 inches out from the tag
    }

    @Override
    public void initialize(){
        // feeding in the distance away from setpoints to PID
        this.translationXController.setSetpoint(0); // zero difference between the current and goal translation
        this.translationYController.setSetpoint(0); 
        this.rotationController.setSetpoint(0);
    }

    @Override
    public void execute(){
        Logger.recordOutput("Commands/alignToTag", true);
        this.translationToPose = this.vision.getSingleTargetTranslation(tagId);
        this.translationToPose = translationToPose.minus(offset); // add the left, right, or center offset
        
        // Calculate chassis speeds from PID 
        double xSpeed = -this.translationXController.calculate(translationToPose.getX());
        double ySpeed = -this.translationYController.calculate(translationToPose.getY());
        double turnSpeed = -this.rotationController.calculate(drivetrain.getRotation().getRadians()); // ambiguity can cause rotation to be off --> use gyro!!

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);

        this.drivetrain.driveVelocity(speeds);
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
    }

    
}
