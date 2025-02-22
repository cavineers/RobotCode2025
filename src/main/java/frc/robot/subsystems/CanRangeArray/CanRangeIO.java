package frc.robot.subsystems.CanRangeArray;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface CanRangeIO {
    @AutoLog
    public static class CanRangeIOInputs {
        public boolean connected = false;
        public double distance = 0;
        public int id = 0;

    }

    public default void updateInputs(CanRangeIOInputs inputs) {
    }
}