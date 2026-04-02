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

        configureDefaultCommands();
        configureBindings();
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

        // --- Operator ---

        operatorController.b().whileTrue(flywheelSubsystem.setVelocity(() -> Units.RPM.of(4000)))
                             .onFalse(Commands.runOnce(() -> flywheelSubsystem.runOpenLoop(0), flywheelSubsystem));

        operatorController.a().whileTrue(Commands.startEnd(
                loaderSubsystem::feed,
                loaderSubsystem::stop,
                loaderSubsystem));

        operatorController.leftBumper().whileTrue(Commands.startEnd(
                () -> intakeSubsystem.runPivotOpenLoop(0.245),
                () -> intakeSubsystem.runPivotOpenLoop(0.0),
                intakeSubsystem));
        operatorController.rightBumper().whileTrue(Commands.startEnd(
                () -> intakeSubsystem.runPivotOpenLoop(-0.245),
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
