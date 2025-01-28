package frc.robot.subsystems.Drivetrain;

import static frc.robot.subsystems.Drivetrain.SwerveDriveConstants.DriveConstants.kOdometryFrequency;
import static frc.robot.subsystems.Drivetrain.SwerveDriveConstants.DriveConstants.kPigeonID;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;

/** IO implementation for Pigeon 2. */
public class GyroPigeonIO implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(kPigeonID);
    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final Queue<Double> yawPositionQueue;
    private final StatusSignal<LinearAcceleration> AccelerationX = pigeon.getAccelerationX();
    private final StatusSignal<LinearAcceleration> AccelerationY = pigeon.getAccelerationY();
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
    private List<Double> combinedAccelList;
    

    public GyroPigeonIO() {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(kOdometryFrequency);
        yawVelocity.setUpdateFrequency(50.0);
        pigeon.optimizeBusUtilization();
        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(yaw::getValueAsDouble);
        combinedAccelList = new ArrayList<>();
    }

    
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
   
        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
        
        double accelX = AccelerationX.getValueAsDouble();
        double accelY = AccelerationY.getValueAsDouble();
        double combinedAccel = Math.sqrt(accelX * accelX + accelY * accelY);

        combinedAccelList.add(combinedAccel);
        inputs.combinedAccel = combinedAccelList.stream().mapToDouble(Double::doubleValue).toArray();
    }
}