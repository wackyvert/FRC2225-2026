package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public record VisionObservation(
    boolean hasTargets,
    double timestampSeconds,
    double yawDeg,
    double pitchDeg,
    Pose2d estimatedRobotPose,
    double ambiguity
) {}
