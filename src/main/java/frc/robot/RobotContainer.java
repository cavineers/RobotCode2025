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

public class RobotContainer {

    private final Lights lights;



    // Controllers
    private final CommandXboxController primaryDriverController = new CommandXboxController(0);
    private final CommandXboxController secondaryDriverController = new CommandXboxController(1);

    
    // Commands

    // Auto chooser
    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
    

        // create non simulated subsystems
        lights = new Lights(); // FILL IN SUPPLIERS
        // Create commands
       
        configureButtonBindings();
        configureNamedCommands();


        // // Set up auto routines for SysIds
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
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

        primaryDriverController.b().whileTrue(endEffector.shootCommand());
        primaryDriverController.a().onTrue(dealgaefier.deployCommand());
        primaryDriverController.a().onFalse(dealgaefier.intakeCommand());
        primaryDriverController.x().onFalse(dealgaefier.shootCommand());

        primaryDriverController.leftBumper().whileTrue(new AlignToPeg(drivetrain, canRangeArray, true));
        primaryDriverController.rightBumper().whileTrue(new AlignToPeg(drivetrain, canRangeArray, false));
      
        secondaryDriverController.povLeft().onTrue(elevator.goToPresetCommand(ElevatorConstants.kRestRotations));
        secondaryDriverController.povUp().onTrue(elevator.goToPresetCommand(ElevatorConstants.kL1Rotations));
        secondaryDriverController.povRight().onTrue(elevator.goToPresetCommand(ElevatorConstants.kL2Rotations));
        secondaryDriverController.povDown().onTrue(elevator.goToPresetCommand(ElevatorConstants.kL3Rotations));
        secondaryDriverController.a().onTrue(elevator.goToPresetCommand(ElevatorConstants.kL4Rotations));
        secondaryDriverController.b().onTrue(elevator.goToPresetCommand(ElevatorConstants.kAlgae1Rotations));

    }

    public void configureNamedCommands(){
         // Register Named Commands
        NamedCommands.registerCommand("pegLeft", new AlignToPeg(drivetrain, canRangeArray, true));
        NamedCommands.registerCommand("pegRight", new AlignToPeg(drivetrain, canRangeArray, false));
        NamedCommands.registerCommand("elevatorL1", elevator.goToPresetCommand(kL1Rotations));
        NamedCommands.registerCommand("elevatorL2", elevator.goToPresetCommand(ElevatorConstants.kL2Rotations));
        NamedCommands.registerCommand("elevatorL3", elevator.goToPresetCommand(ElevatorConstants.kL3Rotations));
        NamedCommands.registerCommand("elevatorL4", elevator.goToPresetCommand(ElevatorConstants.kL4Rotations));
        NamedCommands.registerCommand("elevatorRest", elevator.goToPresetCommand(ElevatorConstants.kRestRotations));
        
    }

    public Command getAutonomousCommand() {

        return autoChooser.get();
    }
}
