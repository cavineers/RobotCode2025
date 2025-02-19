package frc.robot;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIOSpark;
import frc.robot.commands.SystemIdCommands;

public class RobotContainer {

    // Subsystems
    // private final SwerveDriveSubsystem drivetrain;
    private final Elevator elevator;

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);

    // Auto chooser
    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        switch (Constants.currentMode) {
            // Instantiate input/output for their respective modes
            case REAL:
                // drivetrain = new SwerveDriveSubsystem(
                //         new GyroPigeonIO(),
                //         new ModuleIOSpark(0),
                //         new ModuleIOSpark(1),
                //         new ModuleIOSpark(2),
                //         new ModuleIOSpark(3));

                elevator = new Elevator(new ElevatorIOSpark());
                break;
            case SIM:
                // drivetrain = new SwerveDriveSubsystem(
                //         new GyroIO() {},
                //         new ModuleIOSim(),
                //         new ModuleIOSim(),
                //         new ModuleIOSim(),
                //         new ModuleIOSim());

                elevator = new Elevator(new ElevatorIOSim());
                break;
            default:
                // Replay
                // drivetrain = new SwerveDriveSubsystem(
                //         new GyroIO() {},
                //         new ModuleIO() {},
                //         new ModuleIO() {},
                //         new ModuleIO() {},
                //         new ModuleIO() {});

                elevator = new Elevator(new ElevatorIO(){});
                break;
        }
        configureButtonBindings();


        // // Set up auto routines for SysIds
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        // // Set up SysId routines
        // autoChooser.addOption(
        //     "Drive Wheel Radius Characterization", SystemIdCommands.wheelRadiusCharacterization(drivetrain));
        // autoChooser.addOption(
        //     "Drive Simple FF Characterization", SystemIdCommands.feedforwardCharacterization(drivetrain));
        // autoChooser.addOption(
        //     "Drive SysId (Quasistatic Forward)",
        //     drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //     "Drive SysId (Quasistatic Reverse)",
        //     drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption(
        //     "Drive SysId (Dynamic Forward)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //     "Drive SysId (Dynamic Reverse)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        
        // Configure the button bindings
        configureButtonBindings();
  }

    private void configureButtonBindings() {
        // Set the drivetrain default command
        // drivetrain.setDefaultCommand(new SwerveCommand(
        //         drivetrain,
        //         driverController::getLeftY,
        //         driverController::getLeftX,
        //         driverController::getRightX));

        driverController.a().onTrue(elevator.goToPresetCommand(ElevatorConstants.kL1RotationsRotations));
        driverController.b().onTrue(elevator.goToPresetCommand(ElevatorConstants.kL2RotationsRotations));
        driverController.x().onTrue(elevator.goToPresetCommand(ElevatorConstants.kL3RotationsRotations));
        // driverController.y().onTrue(elevator.goToPresetCommand(ElevatorConstants.kL4RotationsRotations));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
