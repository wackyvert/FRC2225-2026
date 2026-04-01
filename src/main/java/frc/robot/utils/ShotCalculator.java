package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.constants.Constants.ShooterConstants;

public class ShotCalculator {
    private final InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();

    public ShotCalculator() {
        // Example Data Points (Distance Meters -> RPM)
        rpmTable.put(1.0, 3000.0);
        rpmTable.put(3.0, 4500.0);
        rpmTable.put(6.0, 5500.0);
    }

    public ShotState calculate(double distanceMeters) {
        double flywheelRPM = rpmTable.get(distanceMeters);
        flywheelRPM = Math.min(ShooterConstants.FLYWHEEL_MAX_RPM, flywheelRPM);
        return new ShotState(flywheelRPM);
    }

    public record ShotState(double flywheelRPM) {}
}
