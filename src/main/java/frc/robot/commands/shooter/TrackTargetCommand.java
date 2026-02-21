package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.Constants.Global;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.subsystems.shooter.HoodSubsystem;
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
    private final HoodSubsystem hood;
    private final ShooterFlywheelSubsystem flywheel;
    private final VisionSubsystem vision;
    private final ShotCalculator calculator;

    // State for Flywheel (since Supplier must return a value)
    private double lastRPM = 0;

    public TrackTargetCommand(TurretSubsystem turret, HoodSubsystem hood, ShooterFlywheelSubsystem flywheel, VisionSubsystem vision, ShotCalculator calculator) {
        System.out.println("[TrackTarget] Constructor called");
        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
        this.vision = vision;
        this.calculator = calculator;
        
        // Requirements are handled by the composed commands, but since we pass subsystems to them:
        // flywheel.setVelocity(...) requires flywheel
        // Commands.run(..., hood) requires hood
        // Commands.run(..., turret) requires turret
        
        addCommands(
            flywheel.setVelocity(this::getFlywheelSetpoint),
            Global.HOOD_ENABLED   ? Commands.run(this::updateHood, hood)     : Commands.none(),
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
            // Return last valid RPM if target lost, or 0 if never found
            System.out.println("[TrackTarget] Commanding Flywheel: " + lastRPM + " RPM");
            return Units.RPM.of(lastRPM);
        } catch (Exception e) {
            System.out.println("[TrackTarget] ERROR in getFlywheelSetpoint: " + e.getMessage());
            e.printStackTrace();
            return Units.RPM.of(0);
        }
    }

    private void updateHood() {
        try {
            VisionObservation obs = vision.getLatestObservation();
            if (obs.hasTargets()) {
                double distance = obs.estimatedRobotPose().getTranslation().getNorm();
                ShotState state = calculator.calculate(distance);
                System.out.println("[TrackTarget] Commanding Hood: " + state.hoodAngleDeg() + " deg");
                hood.setAngleDeg(state.hoodAngleDeg());
            }
        } catch (Exception e) {
            System.out.println("[TrackTarget] ERROR in updateHood: " + e.getMessage());
        }
    }

    private void updateTurret() {
        try {
            VisionObservation obs = vision.getLatestObservation();
            if (obs.hasTargets()) {
                // Camera is on the robot body (not the turret).
                // cameraYaw is measured in robot frame, so the target's angle in robot
                // frame is simply: cameraYaw + camera mounting yaw offset.
                // The turret error is the difference between that and the current turret angle.
                // Apply proportionally to avoid overshoot.
                double targetInRobotFrame = obs.yawDeg() + VisionConstants.CAMERA_YAW_OFFSET_DEG;
                double error = targetInRobotFrame - turret.getAngleDeg();
                double targetTurret = turret.getAngleDeg() + 0.3 * error;
                System.out.println("[TrackTarget] Commanding Turret: " + targetTurret + " deg");
                turret.setAngleDeg(targetTurret);
            }
        } catch (Exception e) {
            System.out.println("[TrackTarget] ERROR in updateTurret: " + e.getMessage());
        }
    }
}