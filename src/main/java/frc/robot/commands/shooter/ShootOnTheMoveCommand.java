package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShotingOnTheFlyConstants;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.field.AllianceFlipUtil;
import frc.robot.utils.field.GeomUtil;
import java.util.function.DoubleSupplier;

/**
 * Shoot-on-the-move command adapted from FRC 2181.
 * Dynamically aims turret and flywheel while the robot is driving,
 * using lookahead calculations to compensate for robot velocity and time-of-flight.
 */
public class ShootOnTheMoveCommand extends Command {
  private static final double TURRET_ASSIST_MARGIN_DEG = 5.0;
  private static final int LOOKAHEAD_ITERATIONS = 17;

  private final SwerveSubsystem drivetrain;
  private final ShooterFlywheelSubsystem flywheel;
  private final TurretSubsystem turret;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / ShotingOnTheFlyConstants.loopPeriodSecs));

  private Rotation2d lastTurretAngle;
  private Rotation2d turretAngle;
  private double turretVelocity;

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double flywheelSpeed) {}

  private LaunchingParameters latestParameters = null;
  private String status = "IDLE";

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.534;
    maxDistance = 7.706;
    phaseDelay = 0.03; // should be .13?

    // Measured flywheel RPM vs horizontal distance to goal center (meters).
    launchFlywheelSpeedMap.put(2.134, 2925.0);
    launchFlywheelSpeedMap.put(2.743, 3210.0);
    launchFlywheelSpeedMap.put(3.048, 3420.0);
    launchFlywheelSpeedMap.put(3.353, 3500.0);
    launchFlywheelSpeedMap.put(3.658, 3740.0);
    launchFlywheelSpeedMap.put(3.962, 3900.0);
    launchFlywheelSpeedMap.put(4.420, 4100.0);
    launchFlywheelSpeedMap.put(4.877, 4350.0);
    launchFlywheelSpeedMap.put(5.791, 4700.0);
    launchFlywheelSpeedMap.put(6.706, 5000.0);

    // Time of flight vs distance (seconds)
    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  public ShootOnTheMoveCommand(
      SwerveSubsystem drivetrain,
      ShooterFlywheelSubsystem flywheel,
      TurretSubsystem turret,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier) {
    this.drivetrain = drivetrain;
    this.flywheel = flywheel;
    this.turret = turret;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    addRequirements(drivetrain, flywheel, turret);
  }

  @Override
  public void initialize() {
    super.initialize();
    lastTurretAngle = Rotation2d.fromDegrees(turret.getAngleDeg());
    SmartDashboard.putBoolean("ShootOTM/Active", true);
    SmartDashboard.putString("ShootOTM/Status", "INITIALIZING");
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void execute() {
    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = drivetrain.getPose();
    ChassisSpeeds robotRelativeVelocity = drivetrain.getRobotVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate turret position in field space
    Pose2d turretPosition =
        estimatedPose.transformBy(
            GeomUtil.toTransform2d(ShotingOnTheFlyConstants.robotToTurret));

    // Designate desired target — default to hub
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    if (isInScoringZone(turretPosition)) {
      target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    }

    if (isInLeftNeutralZone(turretPosition)) {
      target =
          AllianceFlipUtil.apply(
              FieldConstants.LeftBump.nearRightCorner.plus(
                  new Translation2d(0, Inches.of(36.5).in(Meters))));
    }

    if (isInRightNeutralZone(turretPosition)) {
      target =
          AllianceFlipUtil.apply(
              FieldConstants.RightBump.nearLeftCorner.plus(
                  new Translation2d(0, -Inches.of(36.5).in(Meters))));
    }

    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = drivetrain.getFieldVelocity();
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (ShotingOnTheFlyConstants.robotToTurret.getY() * Math.cos(robotAngle)
                    - ShotingOnTheFlyConstants.robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (ShotingOnTheFlyConstants.robotToTurret.getX() * Math.cos(robotAngle)
                    - ShotingOnTheFlyConstants.robotToTurret.getY() * Math.sin(robotAngle));

    // Iterative lookahead: account for robot velocity x time-of-flight
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
      Double sampledTimeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      if (sampledTimeOfFlight == null) {
        stopOutputs("NO_TIME_OF_FLIGHT");
        SmartDashboard.putNumber("ShootOTM/DistanceToTarget", lookaheadTurretToTargetDistance);
        SmartDashboard.putBoolean("ShootOTM/InRange", false);
        return;
      }
      timeOfFlight = sampledTimeOfFlight;
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    turretAngle =
        target.minus(lookaheadPose.getTranslation()).getAngle().minus(estimatedPose.getRotation());
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians()
                / ShotingOnTheFlyConstants.loopPeriodSecs);
    lastTurretAngle = turretAngle;
    latestParameters =
        new LaunchingParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            turretAngle,
            turretVelocity,
            getClampedInterpolatedValue(launchFlywheelSpeedMap, lookaheadTurretToTargetDistance));

    // Command subsystems
    Double targetRPM = getClampedInterpolatedValue(launchFlywheelSpeedMap, lookaheadTurretToTargetDistance);
    if (targetRPM == null) {
      stopOutputs("NO_FLYWHEEL_SETPOINT");
      SmartDashboard.putNumber("ShootOTM/DistanceToTarget", lookaheadTurretToTargetDistance);
      SmartDashboard.putBoolean("ShootOTM/InRange", false);
      return;
    }

    boolean inRange = lookaheadTurretToTargetDistance >= minDistance
        && lookaheadTurretToTargetDistance <= maxDistance;

    if (!inRange) {
      stopOutputs("OUT_OF_RANGE");
      SmartDashboard.putNumber("ShootOTM/DistanceToTarget", lookaheadTurretToTargetDistance);
      SmartDashboard.putNumber("ShootOTM/TurretAngleDeg", turretAngle.getDegrees());
      SmartDashboard.putNumber("ShootOTM/FlywheelRPM", targetRPM);
      SmartDashboard.putBoolean("ShootOTM/InRange", false);
      return;
    }

    flywheel.setVelocitySetpoint(RPM.of(targetRPM));
    // Temporary: leave turret under manual control while we validate SOTM RPM behavior.
    // turret.setAngleDegSetpoint(turretAngle.getDegrees());
    // driveWithTurretAssist(target.minus(lookaheadPose.getTranslation()).getAngle(), turretAngle);
    status = "TRACKING";

    SmartDashboard.putNumber("ShootOTM/DistanceToTarget", lookaheadTurretToTargetDistance);
    SmartDashboard.putNumber("ShootOTM/TurretAngleDeg", turretAngle.getDegrees());
    SmartDashboard.putNumber("ShootOTM/FlywheelRPM", targetRPM);
    SmartDashboard.putBoolean("ShootOTM/InRange", latestParameters.isValid());
    SmartDashboard.putBoolean("ShootOTM/Active", true);
    SmartDashboard.putString("ShootOTM/Status", status);
  }

  public LaunchingParameters getLatestParameters() {
    return latestParameters;
  }

  // --- Zone triggers (from 2181) ---

  public boolean isInScoringZone(Pose2d turretPose) {
    return new Rectangle2d(
            AllianceFlipUtil.apply(new Translation2d(0, 0)),
            AllianceFlipUtil.apply(
                new Translation2d(
                    FieldConstants.LinesVertical.starting, FieldConstants.fieldWidth)))
        .contains(turretPose.getTranslation());
  }

  public boolean isInRightNeutralZone(Pose2d turretPose) {
    return new Rectangle2d(
            AllianceFlipUtil.apply(
                new Translation2d(FieldConstants.LinesVertical.starting, 0)),
            AllianceFlipUtil.apply(
                new Translation2d(
                    FieldConstants.LinesVertical.oppAllianceZone,
                    FieldConstants.LinesHorizontal.center)))
        .contains(turretPose.getTranslation());
  }

  public boolean isInLeftNeutralZone(Pose2d turretPose) {
    return new Rectangle2d(
            AllianceFlipUtil.apply(
                new Translation2d(
                    FieldConstants.LinesVertical.starting,
                    FieldConstants.LinesHorizontal.center)),
            AllianceFlipUtil.apply(
                new Translation2d(
                    FieldConstants.LinesVertical.oppAllianceZone,
                    FieldConstants.fieldWidth)))
        .contains(turretPose.getTranslation());
  }

  @Override
  public void end(boolean interrupted) {
    stopOutputs(interrupted ? "INTERRUPTED" : "ENDED");
    SmartDashboard.putBoolean("ShootOTM/Active", false);
  }

  private void stopOutputs(String newStatus) {
    status = newStatus;
    flywheel.runOpenLoop(0);
    drivetrain.driveFieldOriented(new ChassisSpeeds());
    SmartDashboard.putNumber("ShootOTM/FlywheelRPM", 0);
    SmartDashboard.putString("ShootOTM/Status", status);
  }

  private void driveWithTurretAssist(Rotation2d fieldTargetAngle, Rotation2d requestedTurretAngle) {
    double turretLimitDeg =
        Math.min(Math.abs(ShooterConstants.TURRET_MIN_ANGLE_DEG), Math.abs(ShooterConstants.TURRET_MAX_ANGLE_DEG))
            - TURRET_ASSIST_MARGIN_DEG;
    double requestedDeg = requestedTurretAngle.getDegrees();
    double clampedTurretDeg = Math.max(-turretLimitDeg, Math.min(turretLimitDeg, requestedDeg));
    Rotation2d desiredHeading =
        Math.abs(requestedDeg) > turretLimitDeg
            ? fieldTargetAngle.minus(Rotation2d.fromDegrees(clampedTurretDeg))
            : drivetrain.getHeading();

    drivetrain.driveFieldOriented(
        drivetrain.getTargetSpeeds(
            translationXSupplier.getAsDouble(),
            translationYSupplier.getAsDouble(),
            desiredHeading));
  }

  private static Double getClampedInterpolatedValue(
      InterpolatingDoubleTreeMap map, double key) {
    double clampedKey = Math.max(minDistance, Math.min(maxDistance, key));
    return map.get(clampedKey);
  }
}
