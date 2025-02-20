package frc.robot.subsystems.CanRangeArray;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.CanRangeArray.CanRangeArrayConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;


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
        for (CanRangeIO io : ios) {
            CanRangeIO.CanRangeIOInputs inputs = new CanRangeIO.CanRangeIOInputs();
            io.updateInputs(inputs);
        }

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
        double baselineDistance = side ? inputs[1].distance : inputs[3].distance;

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
        double baselineDistance = side ? inputs[1].distance : inputs[3].distance;

        if (this.isAligned(side)) return 0;
        // Compare the distance of the opposing side to the baseline and run the motors in the corresponding direction
        if (side){
            // Left side
            if (getDifference(inputs[1].distance, baselineDistance) > kDifferenceTolerance && getDifference(inputs[0].distance, baselineDistance) > kDifferenceTolerance){
                // If the dist of both sensors is greater than the baseline sensor move to the right
                return -kAlignmentSpeed;
            }
            if (getDifference(inputs[1].distance, baselineDistance) < kDifferenceTolerance && getDifference(inputs[0].distance, baselineDistance) < kDifferenceTolerance){
                // If the dist of both sensors is less than the baseline sensor move to the left
                return kAlignmentSpeed;
            }
        }else{
            // Right side
            if (getDifference(inputs[3].distance, baselineDistance) > kDifferenceTolerance && getDifference(inputs[2].distance, baselineDistance) > kDifferenceTolerance){
                // If the dist of both sensors is greater than the baseline sensor move to the left
                return kAlignmentSpeed;
            }
            if (getDifference(inputs[3].distance, baselineDistance) < kDifferenceTolerance && getDifference(inputs[2].distance, baselineDistance) < kDifferenceTolerance){
                // If the dist of both sensors is less than the baseline sensor move to the right
                return -kAlignmentSpeed;
            }
        }
        
        return 0;
    }
}