package frc.robot.subsystems.Vision;

import static frc.robot.subsystems.Vision.VisionConstants.*;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final VisionIO[] ios;
    private final VisionConsumer consumer;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    public Vision(VisionConsumer consumer, VisionIO... ios) {
        // Define all the fields
        this.consumer = consumer;
        this.ios = ios;
        this.inputs = new VisionIOInputsAutoLogged[ios.length];
        this.disconnectedAlerts = new Alert[ios.length];

        // Initialize all the fields
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] = new Alert(
                    "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < ios.length; i++) { // update all camera inputs
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/camera" + i, inputs[i]); // log the inputs
        }

        // Initialize the pose values
        List<Pose3d> totalTagPoses = new LinkedList<>();
        List<Pose3d> totalRobotPoses = new LinkedList<>();
        List<Pose3d> totalAcceptedRobotPoses = new LinkedList<>();

        // loop through all cams
        for (int i = 0; i < ios.length; i++) {
            disconnectedAlerts[i].set(!inputs[i].connected);

            // Initialize the pose values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> acceptedRobotPoses = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[i].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[i].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (!rejectPose) {
                    acceptedRobotPoses.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;

                if (i < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[i];
                    if (DriverStation.isEnabled()){
                        angularStdDev = 100000; // Don't use the angle reading when the robot is enabled
                    }else{
                        angularStdDev *= cameraStdDevFactors[i]; // Reduce angular std dev in simulation
                    }
                }

                // Send vision observation
                consumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera data
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(i) + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(i) + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(i) + "/RobotPosesAccepted",
                    acceptedRobotPoses.toArray(new Pose3d[acceptedRobotPoses.size()]));
            totalTagPoses.addAll(tagPoses);
            totalRobotPoses.addAll(robotPoses);
            totalAcceptedRobotPoses.addAll(acceptedRobotPoses);
        }

        // Log summary data
        Logger.recordOutput(
                "Vision/Summary/TagPoses", totalTagPoses.toArray(new Pose3d[totalTagPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPoses", totalRobotPoses.toArray(new Pose3d[totalRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                totalAcceptedRobotPoses.toArray(new Pose3d[totalAcceptedRobotPoses.size()]));
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }

    /**
     * Check if a tag is visible to any camera
     * @param tagId
     * @return
     */
    public boolean hasTag(int tagId) {
        for (int i = 0; i < inputs.length; i++) {
            for (int id : inputs[i].tagIds) { 
                if (id == tagId) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Gets the tag pose
     * @param TagId
     * @return The pose of the tag
     */
    public Pose3d getFieldTagPose(int tagId) {
        return aprilTagLayout.getTagPose(tagId).get();
    }
}
