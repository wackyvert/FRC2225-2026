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

public class VisionSubsystem extends SubsystemBase {

    // Reject single-tag estimates above this ambiguity (0 = perfect, 1 = worst)
    private static final double MAX_AMBIGUITY = 0.2;
    // Reject single-tag estimates from tags farther than this (meters)
    private static final double MAX_SINGLE_TAG_DIST_M = 4.0;

    private final List<CameraRig> cameraRigs;

    // Simulation
    private VisionSystemSim visionSim;
    private Supplier<Pose2d> simPoseSupplier;

    private VisionObservation latestObservation =
            new VisionObservation(false, 0, 0, 0, new Pose2d(), 0);
    private Optional<EstimatedRobotPose> latestEstimate = Optional.empty();

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
        List<CameraSample> samples = cameraRigs.stream()
                .map(rig -> new CameraSample(rig, rig.camera().getLatestResult()))
                .toList();

        latestEstimate = samples.stream()
                .map(sample -> estimatePose(sample.rig(), sample.result()))
                .flatMap(Optional::stream)
                .max(Comparator.comparingInt(estimate -> estimate.targetsUsed.size()));

        latestObservation = samples.stream()
                .filter(sample -> sample.result().hasTargets())
                .map(sample -> {
                    var best = sample.result().getBestTarget();
                    return new VisionObservation(
                            true,
                            sample.result().getTimestampSeconds(),
                            best.getYaw() + sample.rig().yawOffsetDeg(),
                            best.getPitch(),
                            latestEstimate.map(estimate -> estimate.estimatedPose.toPose2d())
                                    .orElse(latestObservation.estimatedRobotPose()),
                            best.getPoseAmbiguity());
                })
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

    private Optional<EstimatedRobotPose> estimatePose(CameraRig rig, PhotonPipelineResult result) {
        if (!result.hasTargets()) return Optional.empty();

        // Prefer multi-tag PnP computed on the coprocessor (most accurate)
        Optional<EstimatedRobotPose> estimate = rig.poseEstimator().estimateCoprocMultiTagPose(result);

        // Fall back to single-tag lowest-ambiguity if coprocessor multi-tag is unavailable
        if (estimate.isEmpty()) {
            estimate = rig.poseEstimator().estimateLowestAmbiguityPose(result);

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

    public PhotonCamera getCamera() {
        return cameraRigs.get(0).camera();
    }

    @Override
    public void simulationPeriodic() {
        if (visionSim != null && simPoseSupplier != null) {
            visionSim.update(simPoseSupplier.get());
        }
    }

    private record CameraRig(
            PhotonCamera camera,
            PhotonPoseEstimator poseEstimator,
            Transform3d robotToCamera,
            double yawOffsetDeg,
            PhotonCameraSim[] cameraSimRef) {
        CameraRig(String name, Transform3d robotToCamera, double yawOffsetDeg, AprilTagFieldLayout tagLayout) {
            this(
                    new PhotonCamera(name),
                    new PhotonPoseEstimator(tagLayout, robotToCamera),
                    robotToCamera,
                    yawOffsetDeg,
                    new PhotonCameraSim[1]);
        }

        void setCameraSim(PhotonCameraSim sim) {
            cameraSimRef[0] = sim;
        }
    }

    private record CameraSample(CameraRig rig, PhotonPipelineResult result) {}
}
