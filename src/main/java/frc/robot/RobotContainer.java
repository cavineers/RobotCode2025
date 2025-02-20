package frc.robot;

import static frc.robot.subsystems.Vision.VisionConstants.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.CanRangeArray.CanRangeArray;
import frc.robot.subsystems.Drivetrain.GyroIO;
import frc.robot.subsystems.Drivetrain.GyroPigeonIO;
import frc.robot.subsystems.Drivetrain.ModuleIO;
import frc.robot.subsystems.Drivetrain.ModuleIOSim;
import frc.robot.subsystems.Drivetrain.ModuleIOSpark;
import frc.robot.subsystems.CanRangeArray.CanRangeIOReal;
import frc.robot.subsystems.CanRangeArray.CanRangeIOSim;
import frc.robot.subsystems.CanRangeArray.CanRangeIO;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.commands.SystemIdCommands;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.Vision.*;

public class RobotContainer {

    // Subsystems
    private final SwerveDriveSubsystem drivetrain;
    private final Vision vision;
    private final CanRangeArray canRangeArray;

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    
    // Commands

    // Auto chooser
    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        switch (Constants.currentMode) {
            // Instantiate input/output for their respective modes
            case REAL:
                drivetrain = new SwerveDriveSubsystem(
                        new GyroPigeonIO(),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3));

                vision = new Vision(
                    drivetrain::addVisionMeasurement,
                    new VisionIOPhoton(frontCameraName, robotToFrontCam));
                    // new VisionIOPhoton(backCameraName, robotToBackCam));

                canRangeArray = new CanRangeArray(
                    new CanRangeIOReal(0),
                    new CanRangeIOReal(1),
                    new CanRangeIOReal(2),
                    new CanRangeIOReal(3)
                );
                

                break;
            case SIM:
                drivetrain = new SwerveDriveSubsystem(
                        new GyroIO() {},
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());

                vision = new Vision(
                    drivetrain::addVisionMeasurement,
                    new VisionIOPhotonSim(frontCameraName, robotToFrontCam, () -> drivetrain.getPose()));
                    // new VisionIOPhotonSim(backCameraName, robotToBackCam, () -> drivetrain.getPose()));

                canRangeArray = new CanRangeArray(new CanRangeIOSim(0), new CanRangeIOSim(1), new CanRangeIOSim(2), new CanRangeIOSim(3));

                break;
            default:
                // Replay
                drivetrain = new SwerveDriveSubsystem(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});

                vision = new Vision(drivetrain::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
                canRangeArray = new CanRangeArray(new CanRangeIO() {}, new CanRangeIO() {}, new CanRangeIO() {}, new CanRangeIO() {});

                break;
        }
        // Create commands
       
        configureButtonBindings();


        // Set up auto routines for SysIds
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        // Set up SysId routines
        autoChooser.addOption(
            "Drive Wheel Radius Characterization", SystemIdCommands.wheelRadiusCharacterization(drivetrain));
        autoChooser.addOption(
            "Drive Simple FF Characterization", SystemIdCommands.feedforwardCharacterization(drivetrain));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        
        // Configure the button bindings
        configureButtonBindings();
  }

    private void configureButtonBindings() {
        // Set the drivetrain default command
        drivetrain.setDefaultCommand(new SwerveCommand(
                drivetrain,
                driverController::getLeftY,
                driverController::getLeftX,
                driverController::getRightX));

        driverController.x().whileTrue(new DriveToPose(drivetrain, this.drivetrain.getClosestReefPoseSide(true, true)));
        driverController.b().whileTrue(new DriveToPose(drivetrain, this.drivetrain.getClosestReefPoseSide(false, true)));
        driverController.y().whileTrue(new AlignToPeg(drivetrain, canRangeArray, false));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
