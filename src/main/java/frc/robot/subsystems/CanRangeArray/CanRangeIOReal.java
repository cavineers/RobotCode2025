package frc.robot.subsystems.CanRangeArray;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.configs.CANrangeConfiguration;

import static frc.robot.subsystems.CanRangeArray.CanRangeArrayConstants.kCanIDs;

public class CanRangeIOReal implements CanRangeIO {

    private final CANrange sensor;
    private StatusSignal<Distance> distanceSignal;
    private int id;
    private int windowSize = 1;
    private double[] previousDistances = new double[windowSize];

    public CanRangeIOReal(int ident) {
        this.sensor = new CANrange(kCanIDs[ident]);
        CANrangeConfiguration config = new CANrangeConfiguration();
        config.FovParams.FOVCenterX = 10;
        this.sensor.getConfigurator().apply(config, 2);
        this.distanceSignal = this.sensor.getDistance();
        this.id = ident;
    }

    public void updateInputs(CanRangeIOInputs inputs) {
        this.distanceSignal.refresh();
        inputs.connected = sensor.isConnected();

        double distance = this.distanceSignal.getValueAsDouble();

        // Shift all elements left and insert the new value at the end
        for (int i = 0; i < windowSize - 1; i++) {
            previousDistances[i] = previousDistances[i + 1];
        }
        previousDistances[windowSize - 1] = distance;

        // Compute the average
        double sum = 0;
        for (double d : previousDistances) {
            sum += d;
        }
        inputs.distance = sum / windowSize;
    }
}