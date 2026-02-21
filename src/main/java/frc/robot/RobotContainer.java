package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants.Global;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.SimpleShootCommand;
import frc.robot.commands.shooter.TrackTargetCommand;
import frc.robot.commands.shooter.TurretAimCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.LoaderSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.ShotCalculator;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

    // Subsystems — vision must be constructed first so it can be passed to swerve
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final SwerveSubsystem swerveSubsystem =
            new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"), visionSubsystem);
    private final ShooterFlywheelSubsystem flywheelSubsystem = new ShooterFlywheelSubsystem();
    // Only instantiate if physically connected — avoids CAN timeouts crashing init
    private final HoodSubsystem hoodSubsystem = Global.HOOD_ENABLED ? new HoodSubsystem() : null;
    private final TurretSubsystem turretSubsystem = Global.TURRET_ENABLED ? new TurretSubsystem() : null;
    private final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    // Utils
    private final ShotCalculator shotCalculator = new ShotCalculator();

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    /**
     * Field-relative drive stream using left stick for translation and right stick X for angular velocity.
     * allianceRelativeControl flips the field orientation automatically for red alliance.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX())
            .withControllerRotationAxis(() -> -driverController.getRightX())
            .deadband(0.1)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    /**
     * Clone of the angular-velocity stream converted to heading-vector control.
     * Right stick X/Y set the target heading angle directly.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
            .withControllerHeadingAxis(() -> -driverController.getRightX(),
                                       () -> -driverController.getRightY())
            .headingWhile(true);

    public RobotContainer() {
        configureDefaultCommands();
        configureBindings();
    }

    private void configureDefaultCommands() {
        // Angular-velocity drive (left stick = translation, right stick X = rotation rate)
        swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOriented(driveAngularVelocity));
    }

    private void configureBindings() {
        // --- Driver ---

        // B: lock wheels in X
        driverController.b().onTrue(Commands.runOnce(swerveSubsystem::lock, swerveSubsystem).repeatedly());

        // Start: zero gyro (re-zero field-forward heading)
        driverController.start().onTrue(Commands.runOnce(swerveSubsystem::zeroGyro, swerveSubsystem));

        // Y: hold to aim only the turret at the hub (drivebase free) — skipped when turret disabled
        if (Global.TURRET_ENABLED) {
            driverController.y().whileTrue(
                    new TurretAimCommand(turretSubsystem, swerveSubsystem, visionSubsystem));
        }

        // Left bumper: hold to snap rotation to face the hub for the current alliance
        driverController.leftBumper().whileTrue(swerveSubsystem.aimAtCommand(
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                        return FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
                    }
                    return FieldConstants.Hub.topCenterPoint.toTranslation2d();
                },
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX()));

        // --- Operator ---

        // Hold A to track target (prepare shot)
        operatorController.a().whileTrue(
                new TrackTargetCommand(turretSubsystem, hoodSubsystem, flywheelSubsystem, visionSubsystem, shotCalculator));

        // Hold B to shoot at fixed 6000 RPM + feed when ready
        operatorController.b().whileTrue(
                new SimpleShootCommand(loaderSubsystem, flywheelSubsystem, hoodSubsystem, turretSubsystem, visionSubsystem));

        // Manual feed / reverse
        operatorController.rightBumper().whileTrue(Commands.run(loaderSubsystem::feed, loaderSubsystem));
        operatorController.leftBumper().whileTrue(Commands.run(loaderSubsystem::reverse, loaderSubsystem));

        // Manual hood jog — skipped when hood disabled
        if (Global.HOOD_ENABLED) {
            operatorController.povUp().whileTrue(Commands.run(
                    () -> hoodSubsystem.setAngleDeg(hoodSubsystem.getAngleDeg() + 0.5), hoodSubsystem));
            operatorController.povDown().whileTrue(Commands.run(
                    () -> hoodSubsystem.setAngleDeg(hoodSubsystem.getAngleDeg() - 0.5), hoodSubsystem));
        }

        // Manual turret jog — skipped when turret disabled
        if (Global.TURRET_ENABLED) {
            operatorController.povRight().whileTrue(Commands.run(
                    () -> turretSubsystem.setAngleDeg(turretSubsystem.getAngleDeg() - 1.0), turretSubsystem));
            operatorController.povLeft().whileTrue(Commands.run(
                    () -> turretSubsystem.setAngleDeg(turretSubsystem.getAngleDeg() + 1.0), turretSubsystem));
        }

        // --- Intake (driver) ---
        // Right bumper: hold to deploy + intake, stow on release
        driverController.rightBumper().whileTrue(intakeSubsystem.intakeSequence())
                .onFalse(intakeSubsystem.stowSequence());
        // Right trigger: hold to outtake (arm stays wherever it is)
        driverController.rightTrigger(0.5).whileTrue(Commands.run(intakeSubsystem::outtake, intakeSubsystem))
                .onFalse(Commands.runOnce(intakeSubsystem::stopRoller, intakeSubsystem));

        // --- Climber (operator) ---
        // Y: extend climber (prep to climb)
       // operatorController.y().onTrue(climberSubsystem.extendCommand());
        // X: retract climber (pull robot up)
        //operatorController.x().onTrue(climberSubsystem.retractCommand());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    /**
     * Test mode: operator right trigger scales 0→1 to 0→MAX_RPM.
     * Current RPM and target RPM are shown on SmartDashboard.
     */
    public Command getFlywheelTestCommand() {
        return flywheelSubsystem.setVelocity(
                () -> {
                    double targetRPM = operatorController.getRightTriggerAxis()
                            * ShooterConstants.FLYWHEEL_MAX_RPM;
                    SmartDashboard.putNumber("Test/Flywheel/TargetRPM", targetRPM);
                    return Units.RPM.of(targetRPM);
                });
    }
}
