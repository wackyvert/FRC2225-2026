package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.shooter.LoaderSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.ShotCalculator;

public class ShootCommand extends ParallelCommandGroup {

    public ShootCommand(
        LoaderSubsystem loader,
        ShooterFlywheelSubsystem flywheel,
        TurretSubsystem turret,
        VisionSubsystem vision,
        ShotCalculator calculator
    ) {
        super(
            new TrackTargetCommand(turret, flywheel, vision, calculator),
            new FeedWhenReadyCommand(loader, flywheel, turret, vision)
        );
    }
}
