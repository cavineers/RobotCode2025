package frc.robot.commands.auto;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CanRangeArray.CanRangeArray;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.subsystems.Drivetrain.SwerveDriveConstants.DriveConstants;

public class AlignToPeg extends Command {

    private SwerveDriveSubsystem drivetrain;
    private CanRangeArray canRangeArray;
    private double goalRotation;
    private PIDController turnController;
    private boolean isLeftSide;

    public AlignToPeg(SwerveDriveSubsystem drivetrain, CanRangeArray canRangeArray, boolean isLeftSide) {
        this.drivetrain = drivetrain;
        this.canRangeArray = canRangeArray;
        this.turnController = new PIDController(DriveConstants.PathPlannerTurnP, 0, 0);
        this.turnController.enableContinuousInput(-Math.PI, Math.PI);
        this.isLeftSide = isLeftSide;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        this.goalRotation = this.drivetrain.getClosestTag().getRotation().getRadians()+Math.PI;
    }

    @Override
    public void execute() {
        Logger.recordOutput("Commands/AlignToPeg", true);
        double currentRotation = drivetrain.getPose().getRotation().getRadians();
        double turnSpeed = turnController.calculate(currentRotation, goalRotation);

        ChassisSpeeds speeds = new ChassisSpeeds(0, this.canRangeArray.calculateAlignmentSpeed(this.isLeftSide), turnSpeed);
        drivetrain.driveVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Commands/AlignToPeg", false);
        this.drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        // Condition to end the command
        return this.canRangeArray.isAligned(this.isLeftSide);
    }
}
