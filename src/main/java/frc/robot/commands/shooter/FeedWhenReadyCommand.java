package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.LoaderSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class FeedWhenReadyCommand extends Command {
    private final LoaderSubsystem loader;
    private final ShooterFlywheelSubsystem flywheel;
    private final HoodSubsystem hood;
    private final TurretSubsystem turret;
    private final VisionSubsystem vision;

    public FeedWhenReadyCommand(LoaderSubsystem loader, ShooterFlywheelSubsystem flywheel, HoodSubsystem hood, TurretSubsystem turret, VisionSubsystem vision) {
        this.loader = loader;
        this.flywheel = flywheel;
        this.hood = hood;
        this.turret = turret;
        this.vision = vision;
        addRequirements(loader);
    }

    @Override
    public void execute() {
        try {
            boolean ready = flywheel.atSpeed() 
                && hood.atSetpoint() 
                && turret.atSetpoint() 
                && vision.getLatestObservation().hasTargets();

            if (ready) {
                System.out.println("[FeedWhenReady] Feeding! (Ready=" + ready + ")");
                loader.feed();
            } else {
                System.out.println("[FeedWhenReady] Waiting... (Flywheel=" + flywheel.atSpeed() + ", Hood=" + hood.atSetpoint() + ", Turret=" + turret.atSetpoint() + ", Vis=" + vision.getLatestObservation().hasTargets() + ")");
                loader.stop();
            }
        } catch (Exception e) {
            System.out.println("[FeedWhenReady] ERROR: " + e.getMessage());
            e.printStackTrace();
        }
    }

    @Override
    public void end(boolean interrupted) {
        loader.stop();
    }
}
