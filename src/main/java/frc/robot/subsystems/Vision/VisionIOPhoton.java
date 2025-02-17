package frc.robot.subsystems.Vision;

import static frc.robot.subsystems.Vision.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * VisionIO implementation for real Photon cameras.
 */
public class VisionIOPhoton implements VisionIO {
    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private PhotonPoseEstimator poseEstimator;

    /**
     * Creates a new VisionIOPhoton camera.
     *
     * @param name
     * @param robotToCamera The 3D position of the camera relative to the robot.
     */
    public VisionIOPhoton(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
        this.poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera);
        this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        List<PoseObservation> poseObservations = new ArrayList<>();
        HashSet<Integer> tagIds = new HashSet<>();
        List<Double> tagDistances = new ArrayList<>();

        for (PhotonPipelineResult result : results) {

            if (result.hasTargets() == false) {
                
                continue;
            }

            // This is where the major calculations are done for the pose (this may be the
            // memory hog)
        
            for (var target : result.getTargets()) {
                tagIds.add(target.getFiducialId());
                tagDistances.add(target.getBestCameraToTarget().getTranslation().getNorm());
            }
            double averageTagDistance = tagDistances.stream().mapToDouble(i -> i).average().orElse(0);
            // Calculate the pose estimations (this may be the memory hog)
            poseObservations.add(
                    new PoseObservation(
                            result.getTimestampSeconds(),
                            poseEstimator.update(result).get().estimatedPose,
                            result.getBestTarget().getPoseAmbiguity(),
                            tagIds.size(),
                            averageTagDistance));
        }
        inputs.tagIds = tagIds.stream().mapToInt(i -> i).toArray();
        inputs.poseObservations = poseObservations.toArray(new PoseObservation[poseObservations.size()]);

    }
}