package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.constants.Constants;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;
    private final VisionSubsystem vision;

    /**
     * Creates the swerve drive from JSON config files in the given deploy directory.
     *
     * @param configDirectory Path to swerve JSON configs (deploy/swerve at runtime).
     * @param vision          VisionSubsystem used to fuse AprilTag pose estimates into odometry.
     */
    public SwerveSubsystem(File configDirectory, VisionSubsystem vision) {
        this.vision = vision;

        // Starting pose based on alliance — updated by auto later, just a reasonable default
        boolean blueAlliance = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Blue;
        Pose2d startingPose = blueAlliance
                ? new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0))
                : new Pose2d(new Translation2d(Meter.of(16), Meter.of(4)), Rotation2d.fromDegrees(180));

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(configDirectory).createSwerveDrive(Constants.MAX_SPEED, startingPose);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(false);   // Only use with angle-setpoint control
        swerveDrive.setCosineCompensator(false);   // Causes sim discrepancies; disable
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

        setupPathPlanner();
    }

    /**
     * Alternate constructor for unit tests / programmatic config (no JSON files needed).
     */
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg,
            VisionSubsystem vision) {
        this.vision = vision;
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, Constants.MAX_SPEED,
                new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)), Rotation2d.fromDegrees(0)));
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        if (Constants.VisionConstants.ENABLE_VISION_POSE_ESTIMATION) {
            vision.getLatestPoseEstimate().ifPresent(estimate -> swerveDrive.addVisionMeasurement(
                    estimate.estimatedPose.toPose2d(),
                    estimate.timestampSeconds,
                    vision.getEstimateStdDevs(estimate)));
        }
    }

    @Override
    public void simulationPeriodic() {}

    // -------------------------------------------------------------------------
    // PathPlanner
    // -------------------------------------------------------------------------

    public void setupPathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = true;
            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotVelocity,
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedforward) {
                            swerveDrive.drive(
                                    speedsRobotRelative,
                                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces());
                        } else {
                            swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0),
                            new PIDConstants(5.0, 0.0, 0.0)),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        return alliance.isPresent() && alliance.get() == Alliance.Red;
                    },
                    this);
        } catch (Exception e) {
            e.printStackTrace();
        }

        PathfindingCommand.warmupCommand().schedule();
    }

    /** Run a named PathPlanner auto. */
    public Command getAutonomousCommand(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    /** Follow a specific PathPlannerPath. */
    public Command followPath(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    /** Pathfind to an arbitrary field pose. */
    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
                swerveDrive.getMaximumChassisVelocity(), 4.0,
                swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
        return AutoBuilder.pathfindToPose(pose, constraints,
                edu.wpi.first.units.Units.MetersPerSecond.of(0));
    }

    /**
     * Drive using 254's SwerveSetpointGenerator (port by PathPlanner) for smoother path following.
     */
    private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeSpeeds)
            throws IOException, ParseException {
        SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
                RobotConfig.fromGUISettings(), swerveDrive.getMaximumChassisAngularVelocity());
        AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
                new SwerveSetpoint(swerveDrive.getRobotVelocity(), swerveDrive.getStates(),
                        DriveFeedforwards.zeros(swerveDrive.getModules().length)));
        AtomicReference<Double> prevTime = new AtomicReference<>();

        return startRun(() -> prevTime.set(Timer.getFPGATimestamp()), () -> {
            double now = Timer.getFPGATimestamp();
            SwerveSetpoint next = setpointGenerator.generateSetpoint(
                    prevSetpoint.get(), robotRelativeSpeeds.get(), now - prevTime.get());
            swerveDrive.drive(next.robotRelativeSpeeds(), next.moduleStates(),
                    next.feedforwards().linearForces());
            prevSetpoint.set(next);
            prevTime.set(now);
        });
    }

    public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
        try {
            return driveWithSetpointGenerator(
                    () -> ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading()));
        } catch (Exception e) {
            DriverStation.reportError(e.toString(), true);
        }
        return Commands.none();
    }

    // -------------------------------------------------------------------------
    // Teleop drive
    // -------------------------------------------------------------------------

    /**
     * Field-relative drive command using angular velocity from right stick.
     * X/Y/rotation suppliers are [-1, 1].
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(() -> swerveDrive.drive(
                SwerveMath.scaleTranslation(new Translation2d(
                        translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                        translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                true,
                false));
    }

    /**
     * Field-relative drive command using a heading vector setpoint (right stick X/Y → angle).
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier headingX, DoubleSupplier headingY) {
        return run(() -> {
            Translation2d scaled = SwerveMath.scaleTranslation(
                    new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
                    scaled.getX(), scaled.getY(),
                    headingX.getAsDouble(), headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity()));
        });
    }

    /**
     * Command that locks the robot's rotation to face a field-space target point
     * while the driver controls translation freely.
     */
    public Command aimAtCommand(Supplier<Translation2d> targetSupplier,
            DoubleSupplier translationX, DoubleSupplier translationY) {
        return run(() -> {
            Translation2d toTarget = targetSupplier.get().minus(getPose().getTranslation());
            driveFieldOriented(getTargetSpeeds(
                    translationX.getAsDouble(), translationY.getAsDouble(), toTarget.getAngle()));
        });
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
    }

    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    /** Lock wheels in X pattern to resist being pushed. */
    public void lock() {
        swerveDrive.lockPose();
    }

    /** Lock wheels in X pattern — as a Command (useful for button bindings). */
    public Command lockCommand() {
        return runOnce(swerveDrive::lockPose);
    }

    public Command driveForward() {
        return run(() -> swerveDrive.drive(new Translation2d(1, 0), 0, false, false))
                .finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
    }

    public Command centerModulesCommand() {
        return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(m -> m.setAngle(0.0)));
    }

    // -------------------------------------------------------------------------
    // Gyro / odometry
    // -------------------------------------------------------------------------

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public void zeroGyroWithAlliance() {
        zeroGyro();
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        }
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    // -------------------------------------------------------------------------
    // Speed / velocity helpers
    // -------------------------------------------------------------------------

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d targetAngle) {
        Translation2d scaled = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(
                scaled.getX(), scaled.getY(),
                targetAngle.getRadians(),
                swerveDrive.getOdometryHeading().getRadians(),
                Constants.MAX_SPEED);
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput,
            double headingX, double headingY) {
        Translation2d scaled = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(
                scaled.getX(), scaled.getY(),
                headingX, headingY,
                swerveDrive.getOdometryHeading().getRadians(),
                Constants.MAX_SPEED);
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        swerveDrive.setChassisSpeeds(speeds);
    }

    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
        swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    }

    // -------------------------------------------------------------------------
    // SysId / characterization
    // -------------------------------------------------------------------------

    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, true),
                3.0, 5.0, 3.0);
    }

    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive),
                3.0, 5.0, 3.0);
    }

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
}
