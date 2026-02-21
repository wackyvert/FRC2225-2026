package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Continuously points the turret at the hub.
 *
 * <p>The camera is mounted on the robot body — NOT on the turret. This means camera
 * yaw is measured in robot frame, independently of where the turret is pointing.
 *
 * <p><b>Primary — odometry geometry:</b> Each loop, the field-frame angle from the
 * robot pose to the hub is computed, then the robot's heading is subtracted to yield
 * the required turret angle in robot frame. Works without camera line-of-sight.
 *
 * <p><b>Camera correction:</b> When a target is visible, the camera provides a direct
 * measurement of the target's angle in robot frame:
 *   {@code targetInRobotFrame = cameraYaw + CAMERA_YAW_OFFSET_DEG}
 * The difference between this measurement and the odometry prediction is the drift
 * error. A fraction of that error is blended in each loop to correct for drift without
 * oscillation.
 */
public class TurretAimCommand extends Command {

    /**
     * Fraction of camera–odometry disagreement applied per loop as a correction.
     * 1.0 = trust camera completely; 0.0 = ignore camera. Start low and raise if
     * odometry drift causes consistent aiming error.
     */
    private static final double CAMERA_CORRECTION_KP = 0.3;

    private final TurretSubsystem turret;
    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;

    public TurretAimCommand(TurretSubsystem turret, SwerveSubsystem swerve, VisionSubsystem vision) {
        this.turret = turret;
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        // ------------------------------------------------------------------
        // Step 1: odometry-based target angle in robot frame
        // ------------------------------------------------------------------
        Translation2d hubPosition = getHubPosition();
        Translation2d robotPosition = swerve.getPose().getTranslation();
        Rotation2d robotHeading = swerve.getHeading();

        // Field-frame angle to hub, then convert to robot frame
        Translation2d toHub = hubPosition.minus(robotPosition);
        double hubFieldAngleDeg = toHub.getAngle().getDegrees();
        double turretTargetDeg = normalise(hubFieldAngleDeg - robotHeading.getDegrees());

        // ------------------------------------------------------------------
        // Step 2: camera correction
        //
        // Camera is on the robot body, so cameraYaw is already in robot frame.
        // targetInRobotFrame = cameraYaw + mounting offset
        // Error = camera measurement − odometry prediction
        // Apply a fraction of that error to stay stable while correcting drift.
        // ------------------------------------------------------------------
        var obs = vision.getLatestObservation();
        if (obs.hasTargets()) {
            double cameraTargetRobotFrame = obs.yawDeg() + VisionConstants.CAMERA_YAW_OFFSET_DEG;
            double driftError = normalise(cameraTargetRobotFrame - turretTargetDeg);
            turretTargetDeg = normalise(turretTargetDeg + CAMERA_CORRECTION_KP * driftError);
        }

        turret.setAngleDeg(turretTargetDeg);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Leave turret at its last commanded angle
    }

    // -------------------------------------------------------------------------

    private Translation2d getHubPosition() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
        }
        return FieldConstants.Hub.topCenterPoint.toTranslation2d();
    }

    /** Wrap an angle in degrees to [-180, 180]. */
    private static double normalise(double deg) {
        deg = deg % 360.0;
        if (deg > 180.0)  deg -= 360.0;
        if (deg < -180.0) deg += 360.0;
        return deg;
    }
}
