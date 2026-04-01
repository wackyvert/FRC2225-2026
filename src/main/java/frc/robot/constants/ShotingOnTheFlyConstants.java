package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class ShotingOnTheFlyConstants {
  // TODO: Measure and update these for YOUR robot's turret position
  // X = forward from robot center, Y = left from robot center, Z = up from floor
  public static Transform3d robotToTurret =
      new Transform3d(
          Inches.of(0).in(Meters),   // forward offset — update to match your turret
          Inches.of(0).in(Meters),   // left offset — update to match your turret
          Inches.of(20).in(Meters),  // height — update to match your turret
          new Rotation3d(0, 0, 0));

  public static final double loopPeriodSecs = 0.02; // matches your Global.LOOP_TIME_S
}
