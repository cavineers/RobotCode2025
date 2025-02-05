package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;

public class ElevatorIOSim implements ElevatorIO {

    private static DCMotor motors = DCMotor.getNeoVortex(2).withReduction(1.0 / ElevatorConstants.kGearRatio);

    private static DIOSim limitSwitch = new DIOSim(ElevatorConstants.kLimitSwitchID);

    PIDController elevPid = new PIDController(ElevatorConstants.kProportionalGainSim,
            ElevatorConstants.kIntegralTermSim, ElevatorConstants.kDerivativeTermSim);

    LoggedNetworkNumber tuningP = new LoggedNetworkNumber("/Tuning/P", ElevatorConstants.kProportionalGainSim);
    LoggedNetworkNumber tuningD = new LoggedNetworkNumber("/Tuning/D", ElevatorConstants.kDerivativeTermSim);

    // xdot = Ax + Bu for state-space systems
    // where the first row represents velocity and the second row represents
    // acceleration
    // influenced by motor torque, resistance, sprocket diameter,
    // and the combined mass of the elevator and its load.
    private static final Matrix<N2, N2> A = MatBuilder.fill(Nat.N2(), Nat.N2(), // A 2x2 system dynamics matrix,
                                                                                // defining how the system evolves
                                                                                // without external inputs.
            0, 1,
            0,
            -motors.KtNMPerAmp
                    / (motors.rOhms
                            * Math.pow(Units.inchesToMeters(ElevatorConstants.kSprocketDiameter), 2)
                            * (ElevatorConstants.kElevatorMassKg + ElevatorConstants.kLoadMassKg)
                            * motors.KvRadPerSecPerVolt));

    private static final Vector<N2> B = // B 2x1 matrix, defining how the system reacts to voltage inputs.
            VecBuilder.fill(
                    0.0, motors.KtNMPerAmp / (Units.inchesToMeters(ElevatorConstants.kSprocketDiameter)
                            * (ElevatorConstants.kElevatorMassKg + ElevatorConstants.kLoadMassKg)));

    private static Vector<N2> X = VecBuilder.fill(0.0, 0.0); // Initial state

    @AutoLogOutput(key = "Elevator/Setpoint")
    private double motorSetpoint = 0;
    private double appliedVolts = 0.0;
    private double inputTorqueCurrent = 0.0;

    public void updateInputs(ElevatorIOInputs inputs) {
        if (this.tuningP.get() != elevPid.getP()) {
            elevPid.setP(this.tuningP.get());
        }
        if (this.tuningD.get() != elevPid.getD()) {
            elevPid.setD(this.tuningD.get());
        }


        for (int i = 0; i < 0.02 / (1.0 / 1000.0); i++) {
            setInputTorqueCurrent(
                    elevPid.calculate(Units.metersToInches(X.get(0)) / kRotationToInches)); // expects rotations --> given meters from state vector
            update(1.0 / 1000.0);
        }
        Logger.recordOutput("Elevator/XPositionM", X.get(0));

        inputs.leftPositionRotations = Units.metersToInches(X.get(0)) / kRotationToInches;
        inputs.leftVelocityRPM = 0;
        inputs.leftAppliedVolts = appliedVolts;
        inputs.leftCurrentAmps = inputTorqueCurrent / motors.KtNMPerAmp;

        inputs.rightPositionRotations = Units.metersToInches(X.get(0)) / kRotationToInches;
        inputs.rightVelocityRPM = 0;
        inputs.rightAppliedVolts = appliedVolts;
        inputs.rightCurrentAmps = inputTorqueCurrent / motors.KtNMPerAmp;

    }

    @AutoLogOutput(key = "Elevator/Error")
    private double getError(){
        return elevPid.getPositionError();
    }

    private void update(double dt) {
        // Clamp the input voltage to the motor
        inputTorqueCurrent = MathUtil.clamp(inputTorqueCurrent, -motors.stallCurrentAmps / 2.0,
                motors.stallCurrentAmps / 2.0);

        // Do some physics calculations to update the state
        Matrix<N2, N1> updatedState = NumericalIntegration.rkdp(
                (Matrix<N2, N1> x, Matrix<N1, N1> u) -> A.times(x)
                        .plus(B.times(u))
                        .plus(VecBuilder.fill(
                                0.0,
                                -9.81)), // gravity term
                ElevatorIOSim.X,
                MatBuilder.fill(Nat.N1(), Nat.N1(), inputTorqueCurrent),
                dt);

        // Update the state
        ElevatorIOSim.X = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));

        // Apply the limits (bounds max and min values)
        ElevatorIOSim.X.set(0, 0, MathUtil.clamp(ElevatorIOSim.X.get(0), ElevatorConstants.kMinRotations,
                ElevatorConstants.kMaxRotations));
        
        if (ElevatorIOSim.X.get(0) == ElevatorConstants.kMinRotations || ElevatorIOSim.X.get(0) == ElevatorConstants.kMaxRotations) {
            ElevatorIOSim.X.set(1,0, 0.0); // stop the elevator (velocity) if it hits the top or bottom
        }
    }

    private void setInputTorqueCurrent(double torqueCurrent) {
        inputTorqueCurrent = torqueCurrent;
        appliedVolts = motors.getVoltage(
                motors.getTorque(inputTorqueCurrent), X.get(1, 0) / ElevatorConstants.kSprocketDiameter);
        appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    }

    public boolean getLimitSwitch() {
        return limitSwitch.getValue();
    }

    @Deprecated
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    public void updateSetpoint(double motorSetpoint) {
        this.motorSetpoint = clipSetpoint(motorSetpoint);
        elevPid.setSetpoint(motorSetpoint);
    }

    public double clipSetpoint(double setpoint) {
        if (motorSetpoint > ElevatorConstants.kMaxRotations) {
            return ElevatorConstants.kMaxRotations;
        } else if (motorSetpoint < ElevatorConstants.kMinRotations) {
            return ElevatorConstants.kMinRotations;
        }
        return setpoint;
    }
}