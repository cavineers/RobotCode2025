package frc.robot.subsystems.Vision;

import static frc.robot.subsystems.Vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

/**
 * VisionIO implementation for real Photon cameras.
 */
public class VisionIOPhoton implements VisionIO {
    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;

    /**
     * Creates a new VisionIOPhoton camera.
     *
     * @param name
     * @param robotToCamera The 3D position of the camera relative to the robot.
     */
    public VisionIOPhoton(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        // read unread results
        Set<Short> tagIds = new HashSet<>(); // Set of all tag IDs HashSet is used to avoid duplicates
        List<PoseObservation> poseObservations = new LinkedList<>(); // List of all pose observations
        for (var result : camera.getAllUnreadResults()) {
            // Add pose observation
            if (result.multitagResult.isPresent()) { // Multitag result
                Logger.recordOutput("Vision/multitagPNP", true);
                var multitagResult = result.multitagResult.get();

                // Calculate robot pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                // Calculate average tag distance to tags
                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                // Add tag IDs
                tagIds.addAll(multitagResult.fiducialIDsUsed);

                // Add observation
                poseObservations.add(
                        new PoseObservation(
                                result.getTimestampSeconds(), // Timestamp
                                robotPose, // 3D pose estimate
                                multitagResult.estimatedPose.ambiguity, // Ambiguity
                                multitagResult.fiducialIDsUsed.size(), // Tag count
                                totalTagDistance / result.targets.size() // Average tag distance
                                )); // Observation type

            } else if (!result.targets.isEmpty()) { // Single tag result
                Logger.recordOutput("Vision/multitagPNP", false);
                var target = result.targets.get(0);

                // Calculate robot pose
                var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(),
                            tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    // Add tag ID
                    tagIds.add((short) target.fiducialId);

                    // Add observation
                    poseObservations.add(
                            new PoseObservation(
                                    result.getTimestampSeconds(), // Timestamp
                                    robotPose, // 3D pose estimate
                                    target.poseAmbiguity, // Ambiguity
                                    1, // Tag count
                                    cameraToTarget.getTranslation().getNorm())); 
                }
            }
            
            // create the translation list
            LinkedList<TagTranslationInformation> tagTranslations = new LinkedList<>();
            for (var target : result.targets) {
                // Save tag translation to logger
                Translation2d translation = target.bestCameraToTarget.getTranslation().toTranslation2d();

                // Add tag translation
                TagTranslationInformation tagTranslation = new TagTranslationInformation(target.fiducialId, translation);
                tagTranslations.add(tagTranslation);
            }
            
            inputs.tagTranslations = new TagTranslationInformation[tagTranslations.size()];

            for (int i = 0; i < tagTranslations.size(); i++) {
                inputs.tagTranslations[i] = tagTranslations.get(i);
            }
        }

        // Save pose observations to inputs object
        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }

        // Save tag IDs to inputs objects
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }
}