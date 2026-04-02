package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.Constants.Global;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.ShotCalculator;
import frc.robot.utils.VisionObservation;
import frc.robot.utils.ShotCalculator.ShotState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class TrackTargetCommand extends ParallelCommandGroup {
    private final TurretSubsystem turret;
    private final ShooterFlywheelSubsystem flywheel;
    private final VisionSubsystem vision;
    private final ShotCalculator calculator;

    // State for Flywheel (since Supplier must return a value)
    private double lastRPM = 0;

    public TrackTargetCommand(TurretSubsystem turret, ShooterFlywheelSubsystem flywheel, VisionSubsystem vision, ShotCalculator calculator) {
        System.out.println("[TrackTarget] Constructor called");
        this.turret = turret;
        this.flywheel = flywheel;
        this.vision = vision;
        this.calculator = calculator;

        addCommands(
            flywheel.setVelocity(this::getFlywheelSetpoint),
            Global.TURRET_ENABLED ? Commands.run(this::updateTurret, turret) : Commands.none()
        );
    }

    private AngularVelocity getFlywheelSetpoint() {
        try {
            VisionObservation obs = vision.getLatestObservation();
            if (obs.hasTargets()) {
                double distance = obs.estimatedRobotPose().getTranslation().getNorm();
                ShotState state = calculator.calculate(distance);
                lastRPM = state.flywheelRPM();
            }
            System.out.println("[TrackTarget] Commanding Flywheel: " + lastRPM + " RPM");
            return Units.RPM.of(lastRPM);
        } catch (Exception e) {
            System.out.println("[TrackTarget] ERROR in getFlywheelSetpoint: " + e.getMessage());
            e.printStackTrace();
            return Units.RPM.of(0);
        }
    }

    private void updateTurret() {
        try {
            VisionObservation obs = vision.getLatestObservation();
            if (obs.hasTargets()) {
                double targetInRobotFrame = obs.yawDeg();
                double error = targetInRobotFrame - turret.getAngleDeg();
                double targetTurret = turret.getAngleDeg() + 0.3 * error;
                System.out.println("[TrackTarget] Commanding Turret: " + targetTurret + " deg");
                turret.setAngleDegF(targetTurret);
            }
        } catch (Exception e) {
            System.out.println("[TrackTarget] ERROR in updateTurret: " + e.getMessage());
        }
    }
}
