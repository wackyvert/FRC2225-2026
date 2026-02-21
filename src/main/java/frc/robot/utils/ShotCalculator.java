package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.constants.Constants.ShooterConstants;

public class ShotCalculator {
    private final InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();

    public ShotCalculator() {
        // Example Data Points (Distance Meters -> Value)
        // Close range
        rpmTable.put(1.0, 3000.0);
        hoodTable.put(1.0, 20.0);

        // Mid range
        rpmTable.put(3.0, 4500.0);
        hoodTable.put(3.0, 40.0);

        // Far range
        rpmTable.put(6.0, 5500.0);
        hoodTable.put(6.0, 55.0);
    }

    public ShotState calculate(double distanceMeters) {
        double flywheelRPM = rpmTable.get(distanceMeters);
        double hoodAngle = hoodTable.get(distanceMeters);
        
        // Clamp to limits
        hoodAngle = Math.max(ShooterConstants.HOOD_MIN_ANGLE_DEG, Math.min(ShooterConstants.HOOD_MAX_ANGLE_DEG, hoodAngle));
        flywheelRPM = Math.min(ShooterConstants.FLYWHEEL_MAX_RPM, flywheelRPM);

        return new ShotState(flywheelRPM, hoodAngle);
    }

    public record ShotState(double flywheelRPM, double hoodAngleDeg) {}
}
