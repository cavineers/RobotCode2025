package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
        public TagTranslationInformation[] tagTranslations = new TagTranslationInformation[0];
    }

    /** 
    * A single observation of a pose from a vision system.
    */
    public static record PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance // in meters
        ) {
    }

    /**
     * The robot relative translation to a single tag.
     */
    public static record TagTranslationInformation(
            int tagId,
            Translation2d translation
        ) {
    }

    public default void updateInputs(VisionIOInputs inputs) {
    }
}