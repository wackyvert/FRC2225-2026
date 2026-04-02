package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.utils.VisionObservation;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

    // Reject single-tag estimates above this ambiguity (0 = perfect, 1 = worst)
    private static final double MAX_AMBIGUITY = 0.2;
    // Reject single-tag estimates from tags farther than this (meters)
    private static final double MAX_SINGLE_TAG_DIST_M = 4.0;
    private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);
    private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0);

    private final List<CameraRig> cameraRigs;

    // Simulation
    private VisionSystemSim visionSim;
    private Supplier<Pose2d> simPoseSupplier;

    private VisionObservation latestObservation =
            new VisionObservation(false, 0, 0, 0, new Pose2d(), 0);
    private Optional<EstimatedRobotPose> latestEstimate = Optional.empty();
    private List<VisionEstimate> latestPoseEstimates = List.of();

    public VisionSubsystem() {
        AprilTagFieldLayout tagLayout = FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout();
        cameraRigs = List.of(
                new CameraRig(
                        VisionConstants.LEFT_CAMERA_NAME,
                        VisionConstants.LEFT_ROBOT_TO_CAMERA,
                        VisionConstants.LEFT_CAMERA_YAW_DEG,
                        tagLayout),
                new CameraRig(
                        VisionConstants.RIGHT_CAMERA_NAME,
                        VisionConstants.RIGHT_ROBOT_TO_CAMERA,
                        VisionConstants.RIGHT_CAMERA_YAW_DEG,
                        tagLayout));

        // Set up simulation camera if running in sim
        if (RobotBase.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(tagLayout);

            SimCameraProperties cameraProps = new SimCameraProperties();
            cameraProps.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProps.setCalibError(0.25, 0.08);
            cameraProps.setFPS(20);
            cameraProps.setAvgLatencyMs(35);
            cameraProps.setLatencyStdDevMs(5);

            for (CameraRig rig : cameraRigs) {
                PhotonCameraSim cameraSim = new PhotonCameraSim(rig.camera(), cameraProps);
                cameraSim.enableRawStream(false);
                cameraSim.enableProcessedStream(false);
                rig.setCameraSim(cameraSim);
                visionSim.addCamera(cameraSim, rig.robotToCamera());
            }
        }
    }

    /**
     * Must be called after swerve subsystem is constructed so we can feed the sim
     * the ground-truth robot pose each cycle.
     */
    public void setSimPoseSupplier(Supplier<Pose2d> poseSupplier) {
        this.simPoseSupplier = poseSupplier;
    }

    @Override
    public void periodic() {
        for (CameraRig rig : cameraRigs) {
            rig.updateUnreadResults();
        }

        latestPoseEstimates = cameraRigs.stream()
                .filter(rig -> rig.estimatedRobotPose().isPresent())
                .map(rig -> new VisionEstimate(rig.estimatedRobotPose().get(), rig.curStdDevs()))
                .toList();

        latestEstimate = latestPoseEstimates.stream()
                .map(VisionEstimate::estimate)
                .max(Comparator.comparingInt(estimate -> estimate.targetsUsed.size()));

        latestObservation = cameraRigs.stream()
                .map(CameraRig::getLatestResult)
                .flatMap(Optional::stream)
                .filter(PhotonPipelineResult::hasTargets)
                .map(result -> {
                    CameraRig rig = cameraRigs.stream()
                            .filter(candidate -> candidate.resultsList().contains(result))
                            .findFirst()
                            .orElse(null);
                    if (rig == null) {
                        return null;
                    }
                    PhotonTrackedTarget best = result.getBestTarget();
                    return new VisionObservation(
                            true,
                            result.getTimestampSeconds(),
                            best.getYaw() + rig.yawOffsetDeg(),
                            best.getPitch(),
                            latestEstimate.map(estimate -> estimate.estimatedPose.toPose2d())
                                    .orElse(latestObservation.estimatedRobotPose()),
                            best.getPoseAmbiguity());
                })
                .filter(obs -> obs != null)
                .min(Comparator.comparingDouble(obs -> Math.abs(obs.yawDeg())))
                .orElseGet(() -> new VisionObservation(
                        false,
                        Timer.getFPGATimestamp(),
                        0,
                        0,
                        latestEstimate.map(estimate -> estimate.estimatedPose.toPose2d())
                                .orElse(latestObservation.estimatedRobotPose()),
                        0));
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

    public List<VisionEstimate> getLatestPoseEstimates() {
        return latestPoseEstimates;
    }

    public PhotonCamera getCamera() {
        return cameraRigs.get(0).camera();
    }

    @Override
    public void simulationPeriodic() {
        if (visionSim != null && simPoseSupplier != null) {
            visionSim.update(simPoseSupplier.get());
        }
    }

    public record VisionEstimate(EstimatedRobotPose estimate, Matrix<N3, N1> stdDevs) {}

    private final class CameraRig {
        private final PhotonCamera camera;
        private final PhotonPoseEstimator poseEstimator;
        private final Transform3d robotToCamera;
        private final double yawOffsetDeg;
        private PhotonCameraSim cameraSim;
        private Matrix<N3, N1> curStdDevs = SINGLE_TAG_STD_DEVS;
        private Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
        private List<PhotonPipelineResult> resultsList = List.of();

        CameraRig(String name, Transform3d robotToCamera, double yawOffsetDeg, AprilTagFieldLayout tagLayout) {
            this.camera = new PhotonCamera(name);
            this.poseEstimator = new PhotonPoseEstimator(tagLayout, robotToCamera);
            this.robotToCamera = robotToCamera;
            this.yawOffsetDeg = yawOffsetDeg;
        }

        PhotonCamera camera() {
            return camera;
        }

        Transform3d robotToCamera() {
            return robotToCamera;
        }

        double yawOffsetDeg() {
            return yawOffsetDeg;
        }

        Matrix<N3, N1> curStdDevs() {
            return curStdDevs;
        }

        Optional<EstimatedRobotPose> estimatedRobotPose() {
            return estimatedRobotPose;
        }

        List<PhotonPipelineResult> resultsList() {
            return resultsList;
        }

        void setCameraSim(PhotonCameraSim sim) {
            this.cameraSim = sim;
        }

        Optional<PhotonPipelineResult> getLatestResult() {
            return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(resultsList.size() - 1));
        }

        void updateUnreadResults() {
            resultsList = RobotBase.isReal()
                    ? camera.getAllUnreadResults()
                    : (cameraSim != null ? cameraSim.getCamera().getAllUnreadResults() : List.of());

            if (resultsList.isEmpty()) {
                estimatedRobotPose = Optional.empty();
                return;
            }

            resultsList = new ArrayList<>(resultsList);
            resultsList.sort(Comparator.comparingDouble(PhotonPipelineResult::getTimestampSeconds));
            updateEstimatedGlobalPose();
        }

        private void updateEstimatedGlobalPose() {
            Optional<EstimatedRobotPose> visionEstimate = Optional.empty();
            for (PhotonPipelineResult result : resultsList) {
                visionEstimate = estimatePose(result);
                updateEstimationStdDevs(visionEstimate, result.getTargets());
            }
            estimatedRobotPose = visionEstimate;
        }

        private Optional<EstimatedRobotPose> estimatePose(PhotonPipelineResult result) {
            Optional<EstimatedRobotPose> estimate = poseEstimator.estimateCoprocMultiTagPose(result);
            if (estimate.isEmpty()) {
                estimate = poseEstimator.estimateLowestAmbiguityPose(result);
            }
            return filterEstimate(estimate);
        }

        private Optional<EstimatedRobotPose> filterEstimate(Optional<EstimatedRobotPose> estimate) {
            if (estimate.isEmpty()) {
                return Optional.empty();
            }

            if (estimate.get().targetsUsed.size() == 1) {
                PhotonTrackedTarget target = estimate.get().targetsUsed.get(0);
                double ambiguity = target.getPoseAmbiguity();
                double dist = target.getBestCameraToTarget().getTranslation().getNorm();
                if (ambiguity > MAX_AMBIGUITY || dist > MAX_SINGLE_TAG_DIST_M) {
                    return Optional.empty();
                }
            }

            Pose2d pose = estimate.get().estimatedPose.toPose2d();
            if (pose.getX() < 0
                    || pose.getX() > FieldConstants.fieldLength
                    || pose.getY() < 0
                    || pose.getY() > FieldConstants.fieldWidth) {
                return Optional.empty();
            }

            return estimate;
        }

        private void updateEstimationStdDevs(
                Optional<EstimatedRobotPose> estimate, List<PhotonTrackedTarget> targets) {
            if (estimate.isEmpty()) {
                curStdDevs = SINGLE_TAG_STD_DEVS;
                return;
            }

            Matrix<N3, N1> estimateStdDevs = SINGLE_TAG_STD_DEVS;
            int numTags = 0;
            double avgDist = 0.0;

            for (PhotonTrackedTarget target : targets) {
                Optional<edu.wpi.first.math.geometry.Pose3d> tagPose =
                        poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
                if (tagPose.isEmpty()) {
                    continue;
                }
                numTags++;
                avgDist += tagPose.get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimate.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                curStdDevs = SINGLE_TAG_STD_DEVS;
                return;
            }

            avgDist /= numTags;
            if (numTags > 1) {
                estimateStdDevs = MULTI_TAG_STD_DEVS;
            }

            if (numTags == 1 && avgDist > 4.0) {
                curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                return;
            }

            curStdDevs = estimateStdDevs.times(1.0 + (avgDist * avgDist / 30.0));
        }
    }
}
