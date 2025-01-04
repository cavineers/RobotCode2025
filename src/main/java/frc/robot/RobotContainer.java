package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.Drivetrain.GyroIO;
import frc.robot.subsystems.Drivetrain.GyroPigeonIO;
import frc.robot.subsystems.Drivetrain.ModuleIO;
import frc.robot.subsystems.Drivetrain.ModuleIOSim;
import frc.robot.subsystems.Drivetrain.ModuleIOSpark;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.commands.SystemIdCommands;

public class RobotContainer {

    // Subsystems
    private final SwerveDriveSubsystem driveTrain;

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);

    // Auto chooser
    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        switch (Constants.currentMode) {
            // Instantiate input/output for their respective modes
            case REAL:
                driveTrain = new SwerveDriveSubsystem(
                        new GyroPigeonIO(),
                        new ModuleIOSpark(0),
                        new ModuleIOSpark(1),
                        new ModuleIOSpark(2),
                        new ModuleIOSpark(3));
                break;
            case SIM:
                driveTrain = new SwerveDriveSubsystem(
                        new GyroIO() {},
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());
                break;
            default:
                // Replay
                driveTrain = new SwerveDriveSubsystem(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});
                break;
        }
        configureButtonBindings();


        // Set up auto routines for SysIds
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        // Set up SysId routines
        autoChooser.addOption(
            "Drive Wheel Radius Characterization", SystemIdCommands.wheelRadiusCharacterization(driveTrain));
        autoChooser.addOption(
            "Drive Simple FF Characterization", SystemIdCommands.feedforwardCharacterization(driveTrain));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)", driveTrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)", driveTrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // Configure the button bindings
        configureButtonBindings();
  }

    private void configureButtonBindings() {
        // Set the drivetrain default command
        driveTrain.setDefaultCommand(new SwerveCommand(
                driveTrain,
                driverController::getLeftY,
                driverController::getLeftX,
                driverController::getRightX));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
