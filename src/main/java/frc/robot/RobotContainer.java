package frc.robot;

import static frc.robot.subsystems.Vision.VisionConstants.*;
import static frc.robot.subsystems.Elevator.ElevatorConstants.kL1Rotations;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.SwerveCommand;


import frc.robot.subsystems.Drivetrain.GyroIO;
import frc.robot.subsystems.Drivetrain.GyroPigeonIO;
import frc.robot.subsystems.Drivetrain.ModuleIO;
import frc.robot.subsystems.Drivetrain.ModuleIOSim;
import frc.robot.subsystems.Drivetrain.ModuleIOSpark;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorIO;
import frc.robot.subsystems.EndEffector.EndEffectorIOSim;
import frc.robot.subsystems.EndEffector.EndEffectorIOSpark;
import frc.robot.subsystems.Lights.Lights;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIOSpark;
import frc.robot.commands.SystemIdCommands;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.Vision.*;

public class RobotContainer {

    // Subsystems
    public final SwerveDriveSubsystem drivetrain;

    public final Vision vision;

    public final EndEffector endEffector;
    public final Elevator elevator;
    public final Lights lights;


    // Controllers
    private final CommandXboxController primaryDriverController = new CommandXboxController(0);
    private final CommandXboxController secondaryDriverController = new CommandXboxController(1);

    
    // Commands
    private final Command autoIntakeCommand;

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
                    new VisionIOPhoton(rightCameraName, robotToRightCamera),
                    new VisionIOPhoton(leftCameraName, robotToLeftCamera));

         
                
                endEffector = new EndEffector(new EndEffectorIOSpark());
                elevator = new Elevator(new ElevatorIOSpark());


                break;
            case SIM:
                drivetrain = new SwerveDriveSubsystem(
                        new GyroIO() {},
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());

                vision = new Vision(
                    drivetrain::addVisionMeasurement);
                    // new VisionIOPhotonSim(frontCameraName, robotToFrontCam, () -> drivetrain.getPose()));
                    // new VisionIOPhotonSim(backCameraName, robotToBackCam, () -> drivetrain.getPose()));

                endEffector = new EndEffector(new EndEffectorIOSim());
                elevator = new Elevator(new ElevatorIOSim());
            

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
                endEffector = new EndEffector(new EndEffectorIO(){});
                elevator = new Elevator(new ElevatorIO(){});

                break;
        }
        lights = new Lights(elevator::getElevatorVelocity, endEffector::isShooting, ()-> false, endEffector::getBumpStop, elevator::isIntakePosition);
        // Create commands
        this.autoIntakeCommand = new AutoIntake(this.endEffector);
       
        configureButtonBindings();
        configureNamedCommands();


        // // Set up auto routines for SysIds
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
                primaryDriverController::getLeftY,
                primaryDriverController::getLeftX,
                primaryDriverController::getRightX,
                () -> elevator.getElevatorPosition() > ElevatorConstants.kL2Rotations - 2 && Math.abs(elevator.getElevatorVelocity()) < 3)); // dummy numbers

        primaryDriverController.b().onTrue(
            Commands.runOnce(() -> {
                if (elevator.isIntakePosition()){
                    endEffector.intakeCommand().schedule();
                } else {
                    endEffector.shootCommand(elevator.getElevatorPosition() > ElevatorConstants.kL3Rotations + 5).schedule();
                }
            })
        );
        primaryDriverController.b().onFalse(endEffector.stopCommand());

        primaryDriverController.povUp().onTrue(elevator.setVoltageCommand(1));
        primaryDriverController.povUp().onFalse(elevator.setVoltageCommand(0));
        primaryDriverController.povDown().onTrue(elevator.setVoltageCommand(-1));
        primaryDriverController.povDown().onFalse(elevator.setVoltageCommand(0));

        primaryDriverController.leftBumper().whileTrue(new AlignToClosestTag(drivetrain, drivetrain.getClosestReefPoseSide(true, true)));
        primaryDriverController.rightBumper().whileTrue(new AlignToClosestTag(drivetrain, drivetrain.getClosestReefPoseSide(false, true)));

        primaryDriverController.povLeft().onTrue(Commands.runOnce(() -> this.drivetrain.zeroHeading()));

        
        secondaryDriverController.povLeft().onTrue(elevator.goToPresetCommand(ElevatorConstants.kRestRotations));
        secondaryDriverController.povUp().onTrue(elevator.goToPresetCommand(ElevatorConstants.kL1Rotations));
        secondaryDriverController.povRight().onTrue(elevator.goToPresetCommand(ElevatorConstants.kL2Rotations));
        secondaryDriverController.povDown().onTrue(elevator.goToPresetCommand(ElevatorConstants.kL3Rotations));
        secondaryDriverController.a().onTrue(elevator.goToPresetCommand(ElevatorConstants.kL4Rotations));
        secondaryDriverController.b().onTrue(elevator.goToPresetCommand(ElevatorConstants.kAlgae1Rotations));
        secondaryDriverController.x().onTrue(elevator.goToPresetCommand(ElevatorConstants.kAlgae2Rotations));
        secondaryDriverController.y().onTrue(Commands.runOnce(() -> this.elevator.resetPosition()));
    }

    public void configureNamedCommands(){
         // Register Named Commands
        NamedCommands.registerCommand("elevatorL1", new AutoElevatorPreset(elevator, ElevatorConstants.kL1Rotations));
        NamedCommands.registerCommand("elevatorL2", new AutoElevatorPreset(elevator, ElevatorConstants.kL2Rotations));
        NamedCommands.registerCommand("elevatorL3", new AutoElevatorPreset(elevator, ElevatorConstants.kL3Rotations));
        NamedCommands.registerCommand("elevatorL4", new AutoElevatorPreset(elevator, ElevatorConstants.kL4Rotations));
        NamedCommands.registerCommand("elevatorRest", new AutoElevatorPreset(elevator, ElevatorConstants.kRestRotations));
        NamedCommands.registerCommand("shoot", new AutoShoot(endEffector));
        NamedCommands.registerCommand("intake", new AutoIntake(endEffector));
        NamedCommands.registerCommand("pegLeft", new AlignToClosestTag(drivetrain, drivetrain.getClosestReefPoseSide(true, true)));
        NamedCommands.registerCommand("pegRight", new AlignToClosestTag(drivetrain, drivetrain.getClosestReefPoseSide(false, true)));
    }

    public Command getAutonomousCommand() {

        return autoChooser.get();
    }
}