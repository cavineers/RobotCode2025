package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.SwerveDriveConstants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;

public class PointToTarget extends Command {
    // This command will point the robot towards the target
    
    // Instance variables
    private SwerveDriveSubsystem drivetrain;
    private Pose2d targetPose;
    private final Supplier<Double> xJoystick, yJoystick;
    private final SlewRateLimiter xLimiter, yLimiter;
    

    private PIDController turnController;


    /**
     * Constructs a new PointToTarget command that points the robot towards the target
     * @param drivetrain The drivetrain subsystem
     * @param targetPose The target pose
     * @param xJoystick The x speed joystick
     * @param yJoystick The y speed joystick
     **/
    public PointToTarget(SwerveDriveSubsystem drivetrain, Pose2d targetPose, Supplier<Double> xJoystick, Supplier<Double> yJoystick){
        // Initialize instance variables
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        this.xJoystick = xJoystick;
        this.yJoystick = yJoystick;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turnController = new PIDController(DriveConstants.PathPlannerTurnP, 0, 0);
        this.turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize(){
        // Log that this command has been scheduled
        Logger.recordOutput("PointToTarget/Target", targetPose);
    }

    @Override
    public void execute(){

        // Calculate the angle to the target
        double angleToTarget = Math.atan2(targetPose.getY() - drivetrain.getPose().getY(), targetPose.getX() - drivetrain.getPose().getX());

        // Calculate the angle difference
        double angleDifference =  drivetrain.getPose().getRotation().getRadians() - angleToTarget;

        // Calculate the turning speed
        double turningSpeed = turnController.calculate(angleDifference);

        // Logs
        Logger.recordOutput("PointToTarget/AngleToTarget", angleToTarget);
        Logger.recordOutput("PointToTarget/AngleDifference", angleDifference);
        Logger.recordOutput("PointToTarget/TurningSpeed", turningSpeed);
        Logger.recordOutput("Commands/pointToTarget", true);



        // Get real-time joystick inputs (SAME AS SWERVECOMMAND)

        double xSpeed = -xJoystick.get();
        double ySpeed = -yJoystick.get();

        // Apply deadband -- compensated for when the joystick value does not return to exactly zero
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;

        // Smooths driving for jerky joystick movement & eases acceleration
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        // Convert to field relative chassis speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed,
            drivetrain.getPose().getRotation());
        
        // Drive at the given speeds
        drivetrain.driveVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted){
        // Log that this command has been unscheduled
        Logger.recordOutput("Commands/pointToTarget", false);
        drivetrain.driveVelocity(new ChassisSpeeds(0, 0, 0));
    }


}
