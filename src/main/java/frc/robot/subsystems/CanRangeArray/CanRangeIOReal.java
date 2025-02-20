package frc.robot.subsystems.CanRangeArray;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;

import static frc.robot.subsystems.CanRangeArray.CanRangeArrayConstants.kCanIDs;

public class CanRangeIOReal implements CanRangeIO {
 
    private final CANrange sensor;
    private StatusSignal<Distance> distanceSignal;

    public CanRangeIOReal(int id) {
        this.sensor = new CANrange(kCanIDs[id]);
        this.distanceSignal = this.sensor.getDistance();

    }
    public void updateInputs(CanRangeIOInputs inputs) {
        this.distanceSignal.refresh();
        inputs.connected = sensor.isConnected();
        inputs.distance = this.distanceSignal.getValueAsDouble();
    }
}