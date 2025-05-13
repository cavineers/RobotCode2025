package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    public final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() { 
        io.updateInputs(inputs); 

        Logger.processInputs("Elevator", inputs);

        Logger.recordOutput("3DMechanisms/Elevator", this.getElevatorStagePoses(inputs.rightPositionRotations));
    }

    /**
     * Will return a command that controls the Elevator open loop **ADDS GRAVITY TERM SO 1Volt -> 1.3V**
     * @param volts
     * @return command
     */
    public Command setVoltageCommand(double volts) {
        this.io.setClosedLoop(false);
        if (Constants.currentMode != Constants.simMode){
            return Commands.run(() -> {
                io.setClosedLoop(false);
                io.setVoltage(volts + kGravityTermSpark);
            }, this);
        }
        return Commands.run(() -> {
            io.setClosedLoop(false);
            io.setVoltage(volts);
        }, this);
    }

    public Command goToPresetCommand(double rotations) {
        return Commands.run(() -> {
            this.io.setClosedLoop(true);
            io.updateSetpoint(rotations);
        }, this);
    }

    public double getElevatorPosition() {
        return inputs.rightPositionRotations;
    }

    @AutoLogOutput(key="Elevator/IsIntakePosition")
    public boolean isIntakePosition(){
        return this.getElevatorPosition() < ElevatorConstants.kIntakeShootBoundry;
    }

    @AutoLogOutput(key="Elevator/IsAtSetpoint")
    public boolean IsAtSetpoint(){
        return Math.abs(this.io.getError()) < kSetPointTolerance;
    }

    public Pose3d getRelativeCoralPose() {
        return this.getElevatorStagePoses(inputs.rightPositionRotations)[2].plus(new Transform3d(Units.inchesToMeters(3), 0.0, Units.inchesToMeters(10.25), new Rotation3d(0.0, Units.degreesToRadians(35), 0.0)));
    }

    private Pose3d[] getElevatorStagePoses(double rotations){
        // Calculate how far the chain has rotated
        double inchesMoved = rotations * ElevatorConstants.kRotationToInches; // This is the change in stage height (S1 and S2 ONLY!!!)
    
        // Apply the proportional constant to stage 3 to find the change for S3


        // Calculate the height of the elevator
        double stage1Height = Units.inchesToMeters(inchesMoved);
        double stage2Height =  2 * stage1Height; // S2 is twice the height of S1
        double stage3Height = stage1Height * 3;
        Logger.recordOutput("3DMechanisms/Elevator/Stage1", stage1Height);
        Logger.recordOutput("3DMechanisms/Elevator/Stage2", stage2Height);
        Logger.recordOutput("3DMechanisms/Elevator/Stage3", stage3Height);


        Pose3d stage1 = new Pose3d(0.0, 0.0, stage1Height, new Rotation3d()).plus(kStage1Position);
        Pose3d stage2 = new Pose3d(0.0, 0.0, stage2Height, new Rotation3d()).plus(kStage2Position);
        Pose3d stage3 = new Pose3d(0.0, 0.0, stage3Height, new Rotation3d()).plus(kStage3Position);
        return new Pose3d[]{stage1, stage2, stage3};
    }

    public double getElevatorVelocity() {
        return inputs.rightVelocityRPM;
    }

    public void resetPosition() {
        this.io.resetPosition();
    }
}