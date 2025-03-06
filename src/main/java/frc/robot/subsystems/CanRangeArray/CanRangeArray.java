package frc.robot.subsystems.CanRangeArray;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.subsystems.CanRangeArray.CanRangeArrayConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


public class CanRangeArray extends SubsystemBase {
    /** Creates a new CanRange subsystem. */
    CanRangeIO[] ios; // OUTER INNER: LEFT RIGHT [0] -> OuterLeft
    CanRangeIOInputsAutoLogged[] inputs;
    Alert[] disconnectedAlerts = new Alert[4];

    public CanRangeArray(CanRangeIO io1, CanRangeIO io2, CanRangeIO io3, CanRangeIO io4) {
        this.ios = new CanRangeIO[] {io1, io2, io3, io4};
        this.inputs = new CanRangeIOInputsAutoLogged[ios.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new CanRangeIOInputsAutoLogged();
        }

        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] = new Alert(
                    "CanRange" + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }


    }

    @Override
    public void periodic() {
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("CanRangeArray/Sensor" + i, inputs[i]); // log the inputs

        }

        Logger.recordOutput("CanRangeArray/IsFinished", this.isAligned(true));

        for (int i = 0; i < ios.length; i++) { // Loop through all cameras
            disconnectedAlerts[i].set(!inputs[i].connected);
        }
    }

    /**
     * Get the distances from all the sensors
     * @return [LeftOuter, LeftInner, RightOuter, RightInner]
     */
    public double[] getDistances() {
        double[] distances = new double[ios.length];
        for (int i = 0; i < ios.length; i++) {
            distances[i] = inputs[i].distance;
        }
        return distances;
    }

    /** 
     * Determine if the sensors are aligned
     * @param side true aligning to left side, false if to right side
     * @return true if the sensors are aligned, false if they are not
     */
    public boolean isAligned(boolean side){
        if (!(Constants.currentMode == Constants.Mode.REAL)) return true;
        double baselineDistance = side ? inputs[3].distance : inputs[1].distance;

        // Checks the left side sensors, (One too far and one close)
        if (side){
            if (getDifference(inputs[0].distance, baselineDistance) > kDifferenceTolerance && getDifference(inputs[1].distance, baselineDistance) < kDifferenceTolerance){
                return true;
            }
        }else{
            // Checks the right side sensors, (One too far and one close)
            if (getDifference(inputs[2].distance, baselineDistance) > kDifferenceTolerance && getDifference(inputs[3].distance, baselineDistance) < kDifferenceTolerance){
                return true;
            }
        }
        return false;
    }

    private double getDifference(double distance1, double distance2){
        return Math.abs(distance1 - distance2);
    }

    /**
     * Calculate the alignment speed for the swerve modules
     * @param side true aligning to left side, false if to right side
     * @return the alignment speed for the swerve modules
     */
    @AutoLogOutput(key = "CanRangeArray/CalculatedSpeed")
    public double calculateAlignmentSpeed(boolean side){
        // Check to see which side the baseline is on
        double baselineDistance = side ?  inputs[3].distance: inputs[1].distance;
        if (this.isAligned(side)) return 0;
        if (baselineDistance > kMaxDistance){
            if (side) 
                return kAlignmentSpeed;
            return -kAlignmentSpeed;
        }
        // Compare the distance of the opposing side to the baseline and run the motors in the corresponding direction
        if (side){
            Logger.recordOutput("CanRangeArray/Side", "Left");
            Logger.recordOutput("CanRangeArray/InnerSensorDist", getDifference(inputs[1].distance, baselineDistance));
            Logger.recordOutput("CanRangeArray/OuterSensorDist", getDifference(inputs[0].distance, baselineDistance));
            // Left side
            if (getDifference(inputs[1].distance, baselineDistance) > kDifferenceTolerance && getDifference(inputs[0].distance, baselineDistance) > kDifferenceTolerance){
                // If the dist of both sensors is greater than the baseline sensor move to the RIGHT
                System.out.println("MOVING RIGHT");
                return -kAlignmentSpeed;
            }
            if (getDifference(inputs[1].distance, baselineDistance) < kDifferenceTolerance && getDifference(inputs[0].distance, baselineDistance) < kDifferenceTolerance){
                // If the dist of both sensors is less than the baseline sensor move to the LEFT
                System.out.println("MOVING LEFT");
                return kAlignmentSpeed;
            }
        }else{
            // Right side
            Logger.recordOutput("CanRangeArray/Side", "Right");
            Logger.recordOutput("CanRangeArray/InnerSensorDist", getDifference(inputs[2].distance, baselineDistance));
            Logger.recordOutput("CanRangeArray/OuterSensorDist", getDifference(inputs[3].distance, baselineDistance));
            if (getDifference(inputs[3].distance, baselineDistance) > kDifferenceTolerance && getDifference(inputs[2].distance, baselineDistance) > kDifferenceTolerance){
                // If the dist of both sensors is greater than the baseline sensor move to the left
                System.out.println("MOVING LEFT");
                return kAlignmentSpeed;
            }
            if (getDifference(inputs[3].distance, baselineDistance) < kDifferenceTolerance && getDifference(inputs[2].distance, baselineDistance) < kDifferenceTolerance){
                // If the dist of both sensors is less than the baseline sensor move to the right
                System.out.println("MOVING Right");
                return -kAlignmentSpeed;
            }
        }
        
        return 0;
    }
}