package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.utils.VisionObservation;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {

    // Reject single-tag estimates above this ambiguity (0 = perfect, 1 = worst)
    private static final double MAX_AMBIGUITY = 0.2;
    // Reject single-tag estimates from tags farther than this (meters)
    private static final double MAX_SINGLE_TAG_DIST_M = 4.0;

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private VisionObservation latestObservation =
            new VisionObservation(false, 0, 0, 0, new Pose2d(), 0);
    private Optional<EstimatedRobotPose> latestEstimate = Optional.empty();

    public VisionSubsystem() {
        camera = new PhotonCamera(VisionConstants.CAMERA_NAME);

        // Use 2-arg constructor — the strategy-based constructor is deprecated for removal in 2026.
        // Estimation is done by calling individual methods per frame.
        poseEstimator = new PhotonPoseEstimator(
                FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout(),
                VisionConstants.ROBOT_TO_CAMERA);
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();

        // Update shooter-targeting observation (camera-relative yaw / pitch)
        if (result.hasTargets()) {
            var best = result.getBestTarget();
            latestObservation = new VisionObservation(
                    true,
                    result.getTimestampSeconds(),
                    best.getYaw(),
                    best.getPitch(),
                    latestObservation.estimatedRobotPose(),
                    best.getPoseAmbiguity());
        } else {
            latestObservation = new VisionObservation(
                    false, Timer.getFPGATimestamp(), 0, 0, latestObservation.estimatedRobotPose(), 0);
        }

        // Update field-pose estimate for odometry fusion
        latestEstimate = estimatePose(result);
    }

    private Optional<EstimatedRobotPose> estimatePose(PhotonPipelineResult result) {
        if (!result.hasTargets()) return Optional.empty();

        // Prefer multi-tag PnP computed on the coprocessor (most accurate)
        Optional<EstimatedRobotPose> estimate = poseEstimator.estimateCoprocMultiTagPose(result);

        // Fall back to single-tag lowest-ambiguity if coprocessor multi-tag is unavailable
        if (estimate.isEmpty()) {
            estimate = poseEstimator.estimateLowestAmbiguityPose(result);

            if (estimate.isPresent()) {
                double ambiguity = estimate.get().targetsUsed.get(0).getPoseAmbiguity();
                double dist = estimate.get().targetsUsed.get(0)
                        .getBestCameraToTarget().getTranslation().getNorm();

                // Reject if tag is too ambiguous or too far away for a reliable single-tag fix
                if (ambiguity > MAX_AMBIGUITY || dist > MAX_SINGLE_TAG_DIST_M) {
                    return Optional.empty();
                }
            }
        }

        // Sanity-check: reject poses outside the field boundary
        if (estimate.isPresent()) {
            Pose2d p = estimate.get().estimatedPose.toPose2d();
            if (p.getX() < 0 || p.getX() > FieldConstants.fieldLength
                    || p.getY() < 0 || p.getY() > FieldConstants.fieldWidth) {
                return Optional.empty();
            }
        }

        return estimate;
    }

    /**
     * Standard deviations for the latest estimate, scaled by tag count and distance.
     * More tags and closer range = tighter deviations = more weight in the pose estimator.
     */
    public Matrix<N3, N1> getEstimateStdDevs(EstimatedRobotPose estimate) {
        int numTags = estimate.targetsUsed.size();
        double avgDist = estimate.targetsUsed.stream()
                .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                .average()
                .orElse(10.0);

        // Scale quadratically with distance; improve linearly with more tags
        double xyStdDev = 0.05 * (avgDist * avgDist) / numTags;
        double thetaStdDev = 0.1 * (avgDist * avgDist) / numTags;
        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    /** Camera-relative yaw/pitch to the best visible target — used by shooter commands. */
    public VisionObservation getLatestObservation() {
        return latestObservation;
    }

    /** Field-pose estimate from AprilTags — used by swerve odometry fusion. Empty if no reliable estimate. */
    public Optional<EstimatedRobotPose> getLatestPoseEstimate() {
        return latestEstimate;
    }
}
