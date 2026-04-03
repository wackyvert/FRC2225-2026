package frc.robot.commands.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.Global;
import frc.robot.subsystems.shooter.LoaderSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class FeedWhenReadyCommand extends Command {
    private static final double READY_DEBOUNCE_S = 0.05;
    private static final double AUTO_FEED_RPM_TOLERANCE = 100.0;

    private final LoaderSubsystem loader;
    private final ShooterFlywheelSubsystem flywheel;
    private final TurretSubsystem turret;
    private final VisionSubsystem vision;
    private final Debouncer readyDebouncer = new Debouncer(READY_DEBOUNCE_S);

    public FeedWhenReadyCommand(LoaderSubsystem loader, ShooterFlywheelSubsystem flywheel, TurretSubsystem turret, VisionSubsystem vision) {
        this.loader = loader;
        this.flywheel = flywheel;
        this.turret = turret;
        this.vision = vision;
        addRequirements(loader);
    }

    @Override
    public void execute() {
        try {
            double rpmError = Math.abs(flywheel.getRPM() - SmartDashboard.getNumber("Flywheel/SetpointRPM", 0.0));
            boolean flywheelReady = rpmError <= AUTO_FEED_RPM_TOLERANCE;
            boolean turretReady = true;
            boolean rawReady = flywheelReady && turretReady;
            boolean ready = readyDebouncer.calculate(rawReady);

            SmartDashboard.putBoolean("Shooter/Ready", ready);
            SmartDashboard.putBoolean("Shooter/FlywheelAtSpeed", flywheelReady);
            SmartDashboard.putBoolean("Shooter/TurretAtSetpoint", turretReady);
            SmartDashboard.putBoolean("Shooter/RawReady", rawReady);
            SmartDashboard.putNumber("Shooter/FlywheelRPMError", rpmError);
            if (ready) {
                loader.feed();
            } else {
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
