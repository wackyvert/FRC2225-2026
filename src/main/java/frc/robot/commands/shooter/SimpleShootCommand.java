package frc.robot.commands.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.LoaderSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Spins the flywheel to a fixed 6000 RPM and feeds once up to speed.
 * Hood and turret are ignored (use for week zero drive-base aiming).
 */
public class SimpleShootCommand extends ParallelCommandGroup {

    public SimpleShootCommand(
            LoaderSubsystem loader,
            ShooterFlywheelSubsystem flywheel,
            HoodSubsystem hood,
            TurretSubsystem turret,
            VisionSubsystem vision) {
        addCommands(
            flywheel.setVelocity(() -> Units.RPM.of(6000)),
            new FeedWhenReadyCommand(loader, flywheel, hood, turret, vision)
        );
    }
}
