package frc.robot;

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
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.commands.shooter.TurretAimCommand;
import frc.robot.commands.shooter.ShootOnTheMoveCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
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
    private final TurretSubsystem turretSubsystem = Global.TURRET_ENABLED ? new TurretSubsystem() : null;
    private final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    // Utils
    private final ShotCalculator shotCalculator = new ShotCalculator();

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
        safeJoystickButton(leftDriveStick, 0, 2).whileTrue(Commands.run(swerveSubsystem::lock, swerveSubsystem));

        // Left stick button 3: zero gyro (re-zero field-forward heading)
        safeJoystickButton(leftDriveStick, 0, 3).onTrue(Commands.runOnce(swerveSubsystem::zeroGyro, swerveSubsystem));

        // Right stick button 1: hold to run shoot-on-the-move in sim/teleop.
        if (Global.TURRET_ENABLED) {
            safeJoystickButton(rightDriveStick, 1, 1).whileTrue(
                    new ShootOnTheMoveCommand(
                            swerveSubsystem,
                            flywheelSubsystem,
                            turretSubsystem,
                            () -> -leftDriveStick.getY(),
                            () -> -leftDriveStick.getX()));
            safeJoystickButton(rightDriveStick, 1, 2).whileTrue(
                    new TurretAimCommand(turretSubsystem, swerveSubsystem, visionSubsystem));
            if (RobotBase.isSimulation()) {
                new Trigger(() -> SmartDashboard.getBoolean("Sim/RunSOTM", false))
                        .whileTrue(new ShootOnTheMoveCommand(
                                swerveSubsystem,
                                flywheelSubsystem,
                                turretSubsystem,
                                () -> -leftDriveStick.getY(),
                                () -> -leftDriveStick.getX()));
            }
        }

        // Left stick button 4: hold to snap rotation to face the hub for the current alliance
        safeJoystickButton(leftDriveStick, 0, 4).whileTrue(swerveSubsystem.aimAtCommand(
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                        return FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
                    }
                    return FieldConstants.Hub.topCenterPoint.toTranslation2d();
                },
                () -> -leftDriveStick.getY(),
                () -> -leftDriveStick.getX()));

        // --- Operator ---

        // B: flywheel 5000 RPM, coasts on release
        operatorController.b().whileTrue(flywheelSubsystem.setVelocity(() -> Units.RPM.of(4100)))
                             .onFalse(Commands.runOnce(() -> flywheelSubsystem.runOpenLoop(0), flywheelSubsystem));

        // A: loader feed open loop, stops on release
        operatorController.a().whileTrue(Commands.startEnd(
                loaderSubsystem::feed,
                loaderSubsystem::stop,
                loaderSubsystem));

        if (Global.TURRET_ENABLED) {
            // Manual turret jog
            operatorController.povRight().onTrue(turretSubsystem.jogDeg(-20.0));
            operatorController.povLeft().onTrue(turretSubsystem.jogDeg(20.0));
        }

        operatorController.povUp().onTrue(intakeSubsystem.deployCommand());
        operatorController.povDown().onTrue(intakeSubsystem.stowCommand());


        // --- Intake manual jog (temporary on operator bumpers) ---
        operatorController.leftBumper().whileTrue(Commands.startEnd(
                () -> intakeSubsystem.runPivotOpenLoop(0.245),
                intakeSubsystem::holdCurrentPivotPosition,
                intakeSubsystem));
        operatorController.rightBumper().whileTrue(Commands.startEnd(
                () -> intakeSubsystem.runPivotOpenLoop(-0.245),
                intakeSubsystem::holdCurrentPivotPosition,
                intakeSubsystem));



        // --- Intake roller (operator) ---
        operatorController.x().whileTrue(Commands.startEnd(
                intakeSubsystem::intake,
                intakeSubsystem::stopRoller,
                intakeSubsystem));
        operatorController.y().whileTrue(Commands.startEnd(
                intakeSubsystem::outtake,
                intakeSubsystem::stopRoller,
                intakeSubsystem));

        /*// Right bumper: hold to deploy + intake, stow on release
        operatorController.y().whileTrue(intakeSubsystem.intakeSequence())
                .onFalse(intakeSubsystem.stowSequence());
        // Right trigger: hold to outtake
        operatorController.x().whileTrue(Commands.startEnd(
                intakeSubsystem::outtake,
                intakeSubsystem::stopRoller,
                intakeSubsystem));*/
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private Trigger safeJoystickButton(CommandJoystick joystick, int port, int button) {
        return new Trigger(() -> DriverStation.isJoystickConnected(port) && joystick.getHID().getRawButton(button));
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
