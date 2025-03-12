package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.LocalADStarAK;
import frc.robot.subsystems.Drivetrain.SwerveDriveConstants.DriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDriveSubsystem extends SubsystemBase {
    public AutoBuilder autoBuilder;

    private SysIdRoutine sysId;

    // April Tag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);


    // Gyro Interface
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    // Define Modules
    private final Module[] modules = new Module[4];

    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro.", AlertType.kError);

    private final SwerveDriveKinematics kinematics = DriveConstants.kSwerveKinematics;

    private Rotation2d gyroRotation = new Rotation2d();

    private SwerveModulePosition[] previousModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroRotation,
            previousModulePositions, new Pose2d());

    public SwerveDriveSubsystem(
            GyroIO gyroIO,
            ModuleIO frontLeftModule,
            ModuleIO frontRightModule,
            ModuleIO backLeftModule,
            ModuleIO backRightModule) {

        // Initialize the Gyro
        this.gyroIO = gyroIO;

        // Initialize modules
        modules[0] = new Module(frontLeftModule, 0);
        modules[1] = new Module(frontRightModule, 1);
        modules[2] = new Module(backLeftModule, 2);
        modules[3] = new Module(backRightModule, 3);

        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Configure the System Identification routine
        this.sysId = new SysIdRoutine(new SysIdRoutine.Config( null, null, null,
                (state) -> Logger.recordOutput("Drivetrain/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

        // Configure the AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveVelocity(speeds), // Method that will drive the robot given ROBOT
                                                                 // RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your
                                                // Constants class
                        new PIDConstants(DriveConstants.PathPlannerDriveP, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(DriveConstants.PathPlannerTurnP, 0.0, 0.0) // Rotation PID constants idk why the default is 5
                ),
                DriveConstants.robotConfig, // ROBOT CONFIGURATION
                this::shouldFlipPose, // Method to determine if the path should be flipped
                this // Reference to this subsystem to set requirements
        );

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
            Logger.recordOutput(
                "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
            });
            PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            }
        );

    }

    @Override
    public void periodic() {
        // Gyro
        gyroIO.updateInputs(gyroInputs); // Update gyro values
        Logger.processInputs("Drivetrain/Gyro", gyroInputs);
        gyroDisconnectedAlert.set(!gyroInputs.connected); // Update gyro alert
        Logger.recordOutput("Odometry/FlipPose", shouldFlipPose());


        // Run the periodics for each module
        for (Module module : modules) {
            module.periodic();
        }

        if (DriverStation.isDisabled()) {
            for (Module module : modules) {
                module.stop();
            }

            // Log empty setpoints when disabled
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        // Calculate the rotation
        for (int modIndex = 0; modIndex < 4; modIndex++) {
            modulePositions[modIndex] = modules[modIndex].getPosition();
            moduleDeltas[modIndex] = new SwerveModulePosition(
                    modulePositions[modIndex].distanceMeters - previousModulePositions[modIndex].distanceMeters,
                    modulePositions[modIndex].angle);
            previousModulePositions[modIndex] = modulePositions[modIndex];
        }

        // Update the current gyro rotation
        if (gyroInputs.connected) {
            gyroRotation = gyroInputs.yawPosition;
        } 
        else {
            // Use the delta from kinematics and mods
            Twist2d delta = kinematics.toTwist2d(moduleDeltas);
            gyroRotation = gyroRotation.plus(new Rotation2d(delta.dtheta));
        }
        poseEstimator.update(gyroRotation, this.getModulePositions());
    }


    /**
     * Runs the drivetrain given a velocity
     * 
     * @param speeds
     */
    public void driveVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", moduleStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Order the modules to setpoints
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(moduleStates[i]);
        }

        // Log optimized setpoints
        Logger.recordOutput("SwerveStates/SetpointsOptimized", moduleStates);
    }

    /**
     * Stops the drivetrain
     * 
     * @param speeds
     */
    public void stop() {
        for (Module module : modules) {
            module.stop();
        }
    }

    /**
     * Supplier to determine if the path should be flipped
     * @return flipped if Red
     */
    public Boolean shouldFlipPose() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        return ally.isPresent() && ally.get() == Alliance.Red;
    }

    /**
     * Get the current velocities of every module
     * 
     * @return SwerveModuleState[] states
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Get the current position of every module
     * 
     * @return SwerveModulePosition[] positions
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    /**
     * Get the current chassis speeds of the robot
     * 
     * @return ChassisSpeeds speeds
     */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Gets the position of each module in radians
     * 
     * @return double[] positions
     */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];

        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }

        return values;
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /**
     * Gets the average velocity of each module in rad/sec
     * 
     * @return double velocitie
     */
    public double getCharacterizationAverageVelocity() {
        double sum = 0;
        for (int i = 0; i < 4; i++) {
            sum += modules[i].getFFCharacterizationVelocity();
        }
        return sum / 4.0;
    }

    /**
     * Gets the current pose of the robot
     * 
     * @return Pose2d pose
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** 
     * Accepts a vision measurement and updates the pose estimator.
    */
    public void addVisionMeasurement(Pose2d poseMeters, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
        if (Constants.currentMode != Constants.Mode.REAL)
            return; // for some reason sim camera is being funky
        
        Pose2d pose;
        pose = new Pose2d(poseMeters.getX(), poseMeters.getY(), 
            DriverStation.isEnabled() ? this.gyroRotation : poseMeters.getRotation());
        poseEstimator.addVisionMeasurement(
            pose, timestamp, visionMeasurementStdDevs);
    }

    public void zeroHeading(){
        System.out.println("RESETTING HEADING");
        this.poseEstimator.resetPosition(gyroRotation, previousModulePositions, new Pose2d(this.getPose().getX(), this.getPose().getY(), new Rotation2d()));
    }

    /**
     * Gets the rotation of the robot
     * 
     * @return Rotation2d rotation
     */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /**
     * Resets the odometry of the robot
     * 
     * @param pose
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(gyroRotation, getModulePositions(), pose);
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    public Pose2d getClosestTag(){
        Pose2d closest = null;
        boolean isRedAlliance = this.shouldFlipPose();
        for (AprilTag tag : aprilTagLayout.getTags()) {
            // Guard clause
            if ((!isRedAlliance && (tag.ID < 17 || tag.ID > 22)) || (isRedAlliance && (tag.ID < 6 || tag.ID > 11))) {
                continue; // Check to make sure the tag is in the correct range for each alliance from field drawing
            }
            Translation2d tagPose = tag.pose.toPose2d().getTranslation(); // Convert to translation
            Translation2d robotPose = this.getPose().getTranslation(); // Convert to translation
             
            double distanceToTag = tagPose.getDistance(robotPose);
            if (closest == null || distanceToTag < closest.getTranslation().getDistance(robotPose)) {
                closest = tag.pose.toPose2d(); // closer tag or first tag
            }
        }
        return closest;
    }
    
    /**
     * @param Flip whether to make the pose left or right of the center
     * @param applyOffset whether to apply the offset to the front bumper and side
     * Returns the closest reef april tag Pose2D to the robot
     * Respective to the FMS alliance color
     */
  
    private Pose2d getClosestReefPose(boolean flip, boolean applyOffset) {
        Pose2d closest = this.getClosestTag();

        if (!applyOffset) {
            return closest;
        }
        // Now add the offset from the robot to the front bumper
        // Get the rotation of the tag
        Rotation2d tagRotation = closest.getRotation();
        // Convert the rotation to a direction vector
        Translation2d direction = new Translation2d(tagRotation.getCos(), tagRotation.getSin());
        // Normalize the direction vector to a magnitude of 1
        direction = direction.div(direction.getNorm());

        Translation2d projectedTranslation = closest.getTranslation().plus(direction.times(DriveConstants.kSideLength / 2.0));
        // Create the new pose
        Pose2d centerPose = new Pose2d(projectedTranslation, closest.getRotation());      
        
        // Create a vector perpendicular to the tag's direction
        Translation2d perpendicular;
        if (!flip) {
            perpendicular = new Translation2d(-centerPose.getRotation().getSin(), centerPose.getRotation().getCos());
        } else {
            perpendicular = new Translation2d(centerPose.getRotation().getSin(), -centerPose.getRotation().getCos());
        }
        // Project out by half the robot width and create new pose
        Translation2d sideTranslation = centerPose.getTranslation().plus(perpendicular.times(0.16)); // distance from center to the reef peg
        return new Pose2d(sideTranslation, centerPose.getRotation().plus(new Rotation2d(Math.PI)));
    }

    public Supplier<Pose2d> getClosestReefPoseSide(boolean leftSide, boolean applyOffset) {
        
        return () -> getClosestReefPose(leftSide, applyOffset);
    }
}
