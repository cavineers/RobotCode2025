package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.lang.Math;
import java.util.function.Supplier;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

public class SwerveCommand extends Command {

    private final SwerveDriveSubsystem swerveSubsystem;

    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveCommand(SwerveDriveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, 
            Supplier<Double> ySpdFunction, 
            Supplier<Double> turningSpdFunction){
        // Instance Variables
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        // Get real-time joystick inputs
        double xSpeed = -xSpdFunction.get();
        double ySpeed = -ySpdFunction.get();
        double turningSpeed = -turningSpdFunction.get();

        // Apply deadband -- compensated for when the joystick value does not return to exactly zero
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // Smooths driving for jerky joystick movement & eases acceleration
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
      
        turningSpeed = turningLimiter.calculate(turningSpeed)
            * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        
        
        // Flipped
        boolean flipped = swerveSubsystem.shouldFlipPose();

        // Convert to field relative chassis speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed,
            flipped ? swerveSubsystem.getPose().getRotation().plus(Rotation2d.fromRadians(Math.PI)) : swerveSubsystem.getPose().getRotation());
        
        // Set the swerve modules to the specified speeds
        swerveSubsystem.driveVelocity(speeds);
        
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}