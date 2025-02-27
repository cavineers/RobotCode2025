package frc.robot.subsystems.Dealgaefier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.Dealgaefier.DealgaefierConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Dealgaefier extends SubsystemBase {
    private final DealgaefierIO io;
    private final DealgaefierIOInputsAutoLogged inputs = new DealgaefierIOInputsAutoLogged();
    private PIDController controller;

    private LoggedNetworkNumber tuningP;
    private LoggedNetworkNumber tuningD;
    private LoggedNetworkNumber tuningG;

    @AutoLogOutput(key="Dealgaefier/Setpoint")
    private double setpoint;


    public Dealgaefier(DealgaefierIO io) {
        this.io = io;
        if (io instanceof DealgaefierIOSpark)
            this.controller = new PIDController(kProportionalGainSpark, kIntegralTermSpark, kDerivativeTermSpark);
        if (io instanceof DealgaefierIOSim)
            this.controller = new PIDController(kProportionalTermSim, 0.0, kDerivativeTermSim);

        this.controller.enableContinuousInput(0, 1);

        if (DealgaefierConstants.kTuningMode && io instanceof DealgaefierIOSpark) {
            tuningP = new LoggedNetworkNumber("Dealgaefier/TuningP", kProportionalGainSpark);
            tuningD = new LoggedNetworkNumber("Dealgaefier/TuningD", kDerivativeTermSpark);
            tuningG = new LoggedNetworkNumber("Dealgaefier/TuningG", kGravityTermSpark);
        }else if (DealgaefierConstants.kTuningMode && io instanceof DealgaefierIOSim) {
            tuningP = new LoggedNetworkNumber("Dealgaefier/TuningP", kProportionalTermSim);
            tuningD = new LoggedNetworkNumber("Dealgaefier/TuningD", kDerivativeTermSim);
            tuningG = new LoggedNetworkNumber("Dealgaefier/TuningG", 0.0);
        }
        this.setpoint = kRestAbsoluteRotations;
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Dealgaefier", inputs);
        
        // Just grab the constant if tuning mode is off
        double gravityTerm = DealgaefierConstants.kTuningMode ? tuningG.get() : kGravityTermSpark;
        double desiredVoltage = this.controller.calculate(inputs.absolutePosition) * -1.0 + (Math.cos(inputs.absolutePosition*2*Math.PI)*gravityTerm);
        Logger.recordOutput("Dealgaefier/DesiredVoltage", desiredVoltage);
        this.io.setDeployVoltage(desiredVoltage);
        if (DealgaefierConstants.kTuningMode){
            this.updatePID();
        }
    }

    public void deploy() {
        this.setpoint = kDeployedAbsoluteRotations;
        this.controller.setSetpoint(kDeployedAbsoluteRotations);
        io.setIntakeVoltage(kIntakeSpeed * 12);
    }

    public void retract() {
        this.setpoint = kRestAbsoluteRotations;
        this.controller.setSetpoint(kRestAbsoluteRotations);
        setIntakeVoltage(0.0);
    }

    public Command setDeployVoltageCommand(double volts) {
        return Commands.run(() -> io.setDeployVoltage(volts), this).finallyDo(interrupted -> io.setDeployVoltage(0));
    }

    public Command setIntakeVoltageCommand(double volts) {
        return Commands.run(() -> io.setIntakeVoltage(volts), this).finallyDo(interrupted -> io.setIntakeVoltage(0));
    }

    public Command deployCommand() {
        return Commands.run(() -> this.deploy(), this).finallyDo(interrupted -> io.setIntakeVoltage(0));
    }

    public Command intakeCommand() {
        return Commands.run(() -> this.retract(), this).finallyDo(interrupted -> io.setIntakeVoltage(0));
    }

    public void setDeployVoltage(double volts) {
        io.setDeployVoltage(volts);
    }

    public void setIntakeVoltage(double volts) {
        io.setIntakeVoltage(volts);
    }

    private void updatePID() {
        double currentP = this.controller.getP();
        double currentD = this.controller.getD();

        if (currentP != this.tuningP.get() || currentD != this.tuningD.get()){
            this.controller.setPID(this.tuningP.get(), 0, this.tuningD.get());
        }
    }
}