package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants.Global;
import frc.robot.commands.shooter.FeedWhenReadyCommand;
import frc.robot.commands.shooter.ShootOnTheMoveCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.LoaderSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {
    private static final InterpolatingDoubleTreeMap AUTO_SHOT_RPM_MAP = new InterpolatingDoubleTreeMap();

    static {
        AUTO_SHOT_RPM_MAP.put(2.134, 2925.0);
        AUTO_SHOT_RPM_MAP.put(2.743, 3210.0);
        AUTO_SHOT_RPM_MAP.put(3.048, 3420.0);
        AUTO_SHOT_RPM_MAP.put(3.353, 3500.0);
        AUTO_SHOT_RPM_MAP.put(3.658, 3740.0);
        AUTO_SHOT_RPM_MAP.put(3.962, 3900.0);
        AUTO_SHOT_RPM_MAP.put(4.420, 4100.0);
        AUTO_SHOT_RPM_MAP.put(4.877, 4350.0);
        AUTO_SHOT_RPM_MAP.put(5.791, 4700.0);
        AUTO_SHOT_RPM_MAP.put(6.706, 5000.0);
    }

    // Subsystems — vision must be constructed first so it can be passed to swerve
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final SwerveSubsystem swerveSubsystem =
            new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"), visionSubsystem);
    private final ShooterFlywheelSubsystem flywheelSubsystem = new ShooterFlywheelSubsystem();
    private final TurretSubsystem turretSubsystem = Global.TURRET_ENABLED ? new TurretSubsystem() : null;
    private final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    // Controllers
    private final CommandJoystick leftDriveStick = new CommandJoystick(0);
    private final CommandJoystick rightDriveStick = new CommandJoystick(1);
    private final CommandXboxController operatorController = new CommandXboxController(2);

    /*
     * Field-relative drive stream using left stick for translation and right stick X for angular velocity.
     * allianceRelativeControl flips the field orientation automatically for red alliance.*/
     
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                    () -> -leftDriveStick.getY(),
                    () -> -leftDriveStick.getX())
            .withControllerRotationAxis(() -> -rightDriveStick.getX())
            .deadband(0.1)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    /*
     * Clone of the angular-velocity stream converted to heading-vector control.
     * Right stick X/Y set the target heading angle directly.*/
     
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
            .withControllerHeadingAxis(() -> -rightDriveStick.getX(),
                                       () -> -rightDriveStick.getY())
            .headingWhile(true);

    public RobotContainer() {
        // Wire vision sim to swerve's ground-truth pose
        visionSubsystem.setSimPoseSupplier(swerveSubsystem::getPose);

        if (RobotBase.isSimulation()) {
            SmartDashboard.putBoolean("Sim/RunSOTM", false);
        }

        registerNamedCommands();
        configureDefaultCommands();
        configureBindings();
    }

    private void registerNamedCommands() {
        if (Global.TURRET_ENABLED) {
            NamedCommands.registerCommand(
                    "ShootOnTheMove",
                    Commands.parallel(
                            new ShootOnTheMoveCommand(
                                    swerveSubsystem,
                                    flywheelSubsystem,
                                    turretSubsystem,
                                    () -> 0.0,
                                    () -> 0.0),
                            new FeedWhenReadyCommand(
                                    loaderSubsystem,
                                    flywheelSubsystem,
                                    turretSubsystem,
                                    visionSubsystem)));
        }
    }

    private void configureDefaultCommands() {
        // Angular-velocity drive (left stick = translation, right stick X = rotation rate)
        swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOriented(driveAngularVelocity));
    }

    private void configureBindings() {
        // --- Driver ---

        safeJoystickButton(leftDriveStick, 0, 1).whileTrue(Commands.startEnd(
                () -> climberSubsystem.runOpenLoop(0.35),
                () -> climberSubsystem.runOpenLoop(0.0),
                climberSubsystem));
        safeJoystickButton(leftDriveStick, 0, 2).whileTrue(Commands.startEnd(
                () -> climberSubsystem.runOpenLoop(-0.35),
                () -> climberSubsystem.runOpenLoop(0.0),
                climberSubsystem));
        safeJoystickButton(rightDriveStick, 1, 1).whileTrue(
                flywheelSubsystem.setVelocity(() -> Units.RPM.of(4800)))
                .onFalse(Commands.runOnce(() -> flywheelSubsystem.runOpenLoop(0), flywheelSubsystem));
        if (Global.TURRET_ENABLED) {
            safeJoystickButton(rightDriveStick, 1, 2).whileTrue(
                    new ShootOnTheMoveCommand(
                            swerveSubsystem,
                            flywheelSubsystem,
                            turretSubsystem,
                            () -> -leftDriveStick.getY(),
                            () -> -leftDriveStick.getX()));
        }
        safeJoystickButton(rightDriveStick, 1, 6).whileTrue(Commands.startEnd(
                () -> climberSubsystem.runOpenLoop(-0.35),
                () -> climberSubsystem.runOpenLoop(0.0),
                climberSubsystem));

        // --- Operator ---

        operatorController.b().whileTrue(flywheelSubsystem.setVelocity(() -> Units.RPM.of(3420)))
                             .onFalse(Commands.runOnce(() -> flywheelSubsystem.runOpenLoop(0), flywheelSubsystem));

        operatorController.a().whileTrue(Commands.startEnd(
                loaderSubsystem::feed,
                loaderSubsystem::stop,
                loaderSubsystem));

        if (Global.TURRET_ENABLED) {
            operatorController.povRight().onTrue(turretSubsystem.jogDeg(-10.0));
            operatorController.povLeft().onTrue(turretSubsystem.jogDeg(10.0));
        }

        operatorController.leftBumper().whileTrue(Commands.startEnd(
                () -> intakeSubsystem.runPivotOpenLoop(0.345),
                () -> intakeSubsystem.runPivotOpenLoop(0.0),
                intakeSubsystem));
        operatorController.rightBumper().whileTrue(Commands.startEnd(
                () -> intakeSubsystem.runPivotOpenLoop(-0.345),
                () -> intakeSubsystem.runPivotOpenLoop(0.0),
                intakeSubsystem));

        operatorController.y().whileTrue(Commands.startEnd(
                intakeSubsystem::intake,
                intakeSubsystem::stopRoller,
                intakeSubsystem));
        operatorController.x().whileTrue(Commands.startEnd(
                intakeSubsystem::outtake,
                intakeSubsystem::stopRoller,
                intakeSubsystem));
    }

    public Command getAutonomousCommand() {
        if (Global.TURRET_ENABLED) {
            return Commands.parallel(
                    new TurretAimCommand(turretSubsystem, swerveSubsystem, visionSubsystem),
                    flywheelSubsystem.setVelocity(() -> Units.RPM.of(getAutoShotRpm())),
                    new FeedWhenReadyCommand(
                            loaderSubsystem,
                            flywheelSubsystem,
                            turretSubsystem,
                            visionSubsystem));
        }
        return Commands.none();
    }

    private Trigger safeJoystickButton(CommandJoystick joystick, int port, int button) {
        return new Trigger(() -> DriverStation.isJoystickConnected(port) && joystick.getHID().getRawButton(button));
    }

    private double getAutoShotRpm() {
        Translation2d hubPosition = DriverStation.getAlliance()
                .filter(alliance -> alliance == DriverStation.Alliance.Red)
                .map(alliance -> frc.robot.constants.FieldConstants.Hub.oppTopCenterPoint.toTranslation2d())
                .orElse(frc.robot.constants.FieldConstants.Hub.topCenterPoint.toTranslation2d());
        double distanceMeters = swerveSubsystem.getPose().getTranslation().getDistance(hubPosition);
        return AUTO_SHOT_RPM_MAP.get(distanceMeters);
    }

    /**
     * Test mode: operator right trigger scales 0→1 to 0→MAX_RPM.
     * Current RPM and target RPM are shown on SmartDashboard.
     */
    /*public Command getFlywheelTestCommand() {
        return flywheelSubsystem.setVelocity(
                () -> {
                    double targetRPM = operatorController.getRightTriggerAxis()
                            * ShooterConstants.FLYWHEEL_MAX_RPM;
                    SmartDashboard.putNumber("Test/Flywheel/TargetRPM", targetRPM);
                    return Units.RPM.of(targetRPM);
                });*/
    
}
