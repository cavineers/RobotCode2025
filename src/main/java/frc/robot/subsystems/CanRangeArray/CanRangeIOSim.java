package frc.robot.subsystems.CanRangeArray;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import static frc.robot.subsystems.CanRangeArray.CanRangeArrayConstants.*;

public class CanRangeIOSim implements CanRangeIO {
 
    private int id;

    public CanRangeIOSim(int ident) {
        this.id = ident;

    }
    public void updateInputs(CanRangeIOInputs inputs) {
        inputs.connected = true;
        inputs.distance = 0;
    }
}