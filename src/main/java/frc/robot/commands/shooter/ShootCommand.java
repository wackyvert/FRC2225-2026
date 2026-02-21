package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.LoaderSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.ShotCalculator;

public class ShootCommand extends ParallelCommandGroup {
    
    public ShootCommand(
        LoaderSubsystem loader, 
        ShooterFlywheelSubsystem flywheel, 
        HoodSubsystem hood, 
        TurretSubsystem turret, 
        VisionSubsystem vision, 
        ShotCalculator calculator
    ) {
        super(
            // Commands to run in parallel:
            // 1. Aim (Hood, Turret, Flywheel Speed)
            new TrackTargetCommand(turret, hood, flywheel, vision, calculator),
            // 2. Feed (Loader) when ready
            new FeedWhenReadyCommand(loader, flywheel, hood, turret, vision)
        );
    }
}
