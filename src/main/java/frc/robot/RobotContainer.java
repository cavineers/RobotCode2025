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

import frc.robot.subsystems.Dealgaefier.Dealgaefier;
import frc.robot.subsystems.Dealgaefier.DealgaefierIO;
import frc.robot.subsystems.Dealgaefier.DealgaefierIOSim;
import frc.robot.subsystems.Dealgaefier.DealgaefierIOSpark;
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
import frc.robot.commands.auto.AutoShoot;
import frc.robot.commands.auto.AutoIntake;

public class RobotContainer {

    // Subsystems
    public final SwerveDriveSubsystem drivetrain;

    public final Dealgaefier dealgaefier;

    public final Vision vision;
    public final CanRangeArray canRangeArray;

    public final EndEffector endEffector;
    public final Elevator elevator;
    public final Lights lights;


    // Controllers
    private final CommandXboxController primaryDriverController = new CommandXboxController(0);
    private final CommandXboxController secondaryDriverController = new CommandXboxController(1);

    
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


                dealgaefier = new Dealgaefier(new DealgaefierIOSpark());

                vision = new Vision(
                    drivetrain::addVisionMeasurement);
                    // new VisionIOPhoton(frontCameraName, robotToFrontCam));
                    // new VisionIOPhoton(backCameraName, robotToBackCam));

                canRangeArray = new CanRangeArray(
                    new CanRangeIOReal(0),
                    new CanRangeIOReal(1),
                    new CanRangeIOReal(2),
                    new CanRangeIOReal(3)
                );
                
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

                dealgaefier = new Dealgaefier(new DealgaefierIOSim());

                vision = new Vision(
                    drivetrain::addVisionMeasurement);
                    // new VisionIOPhotonSim(frontCameraName, robotToFrontCam, () -> drivetrain.getPose()));
                    // new VisionIOPhotonSim(backCameraName, robotToBackCam, () -> drivetrain.getPose()));

                canRangeArray = new CanRangeArray(new CanRangeIOSim(0), new CanRangeIOSim(1), new CanRangeIOSim(2), new CanRangeIOSim(3));

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


                dealgaefier = new Dealgaefier(new DealgaefierIO(){});     

                vision = new Vision(drivetrain::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
                canRangeArray = new CanRangeArray(new CanRangeIO() {}, new CanRangeIO() {}, new CanRangeIO() {}, new CanRangeIO() {});
                endEffector = new EndEffector(new EndEffectorIO(){});
                elevator = new Elevator(new ElevatorIO(){});

                break;
        }
        lights = new Lights(elevator::getElevatorVelocity, endEffector::isShooting, ()-> false, endEffector::getBumpStop, elevator::isIntakePosition);
        // Create commands
       
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
                primaryDriverController::getRightX));

        primaryDriverController.b().onTrue(
            Commands.runOnce(() -> {
                if (elevator.isIntakePosition()){
                    endEffector.intakeCommand().schedule();
                } else {
                    endEffector.shootCommand().schedule();
                }
            })
        );
        primaryDriverController.b().onFalse(endEffector.stopCommand());

        primaryDriverController.rightTrigger(0.85).onTrue(
            Commands.runOnce(() -> {
                if (dealgaefier.getDeployed() == false){
                    dealgaefier.deployCommand().schedule();
                } else {
                    dealgaefier.retractCommand().schedule();
                }
            })
        );

        primaryDriverController.leftTrigger(0.85).onTrue(dealgaefier.shootCommand());
        primaryDriverController.leftTrigger(0.85).onFalse(dealgaefier.retractCommand());

        primaryDriverController.povUp().onTrue(elevator.setVoltageCommand(1));
        primaryDriverController.povUp().onFalse(elevator.setVoltageCommand(0));
        primaryDriverController.povDown().onTrue(elevator.setVoltageCommand(-1));
        primaryDriverController.povDown().onFalse(elevator.setVoltageCommand(0));

        primaryDriverController.leftBumper().whileTrue(new AlignToPeg(drivetrain, canRangeArray, true));
        primaryDriverController.rightBumper().whileTrue(new AlignToPeg(drivetrain, canRangeArray, false));

        primaryDriverController.povLeft().onTrue(Commands.runOnce(() -> this.drivetrain.zeroHeading()));
      
        secondaryDriverController.povLeft().onTrue(elevator.goToPresetCommand(ElevatorConstants.kRestRotations));
        secondaryDriverController.povUp().onTrue(elevator.goToPresetCommand(ElevatorConstants.kL1Rotations));
        secondaryDriverController.povRight().onTrue(elevator.goToPresetCommand(ElevatorConstants.kL2Rotations));
        secondaryDriverController.povDown().onTrue(elevator.goToPresetCommand(ElevatorConstants.kL3Rotations));
        secondaryDriverController.a().onTrue(elevator.goToPresetCommand(ElevatorConstants.kL4Rotations));
        secondaryDriverController.b().onTrue(elevator.goToPresetCommand(ElevatorConstants.kAlgae1Rotations));

    }

    public void configureNamedCommands(){
         // Register Named Commands
        Command l2Command = new AutoElevatorPreset(elevator, ElevatorConstants.kL2Rotations);
        NamedCommands.registerCommand("pegLeft", AutoHelpers.alignToPeg(true));
        NamedCommands.registerCommand("pegRight", AutoHelpers.alignToPeg(false));
        NamedCommands.registerCommand("elevatorL1", new AutoElevatorPreset(elevator, ElevatorConstants.kL1Rotations));
        NamedCommands.registerCommand("elevatorL2", new AutoElevatorPreset(elevator, ElevatorConstants.kL2Rotations));
        NamedCommands.registerCommand("elevatorL3", new AutoElevatorPreset(elevator, ElevatorConstants.kL3Rotations));
        NamedCommands.registerCommand("elevatorL4", new AutoElevatorPreset(elevator, ElevatorConstants.kL4Rotations));
        NamedCommands.registerCommand("elevatorRest", new AutoElevatorPreset(elevator, ElevatorConstants.kRestRotations));
        NamedCommands.registerCommand("shoot", new AutoShoot(endEffector));
        NamedCommands.registerCommand("intake", new AutoIntake(endEffector));

        
    }

    public Command getAutonomousCommand() {

        return autoChooser.get();
    }
}