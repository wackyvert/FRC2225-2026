package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class Constants {
    // Maximum robot translation speed in m/s - tune after characterization
    public static final double MAX_SPEED = 8.5;

    public static final class Global {
        public static final double LOOP_TIME_S = 0.020;

        // Set false for mechanisms that are not yet installed / functional.
        // FeedWhenReadyCommand and TrackTargetCommand skip disabled mechanisms automatically.
        public static final boolean HOOD_ENABLED   = false;
        public static final boolean TURRET_ENABLED = false;
    }

    public static final class VisionConstants {
        /** Set false to disable vision-odometry fusion (e.g. if camera is unreliable at an event). */
        public static final boolean ENABLE_VISION_POSE_ESTIMATION = false;

        public static final String CAMERA_NAME = "shooter_cam";

        // Physical camera mounting — tune these to match your actual robot
        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(20);
        public static final double CAMERA_PITCH_RAD = Units.degreesToRadians(30); // positive = tilted up
        public static final double CAMERA_FORWARD_METERS = 0.0; // distance forward from robot center
        public static final double CAMERA_LEFT_METERS = 0.0;    // distance left from robot center
        // Yaw offset of the camera relative to robot-forward (degrees, positive = rotated left).
        // e.g. if camera faces 10° right of robot-forward, set -10.0
        public static final double CAMERA_YAW_OFFSET_DEG = 0.0;

        // Combined robot-to-camera transform used by PhotonPoseEstimator
        public static final edu.wpi.first.math.geometry.Transform3d ROBOT_TO_CAMERA =
                new edu.wpi.first.math.geometry.Transform3d(
                        new edu.wpi.first.math.geometry.Translation3d(
                                CAMERA_FORWARD_METERS, CAMERA_LEFT_METERS, CAMERA_HEIGHT_METERS),
                        new edu.wpi.first.math.geometry.Rotation3d(0.0, CAMERA_PITCH_RAD, 0.0));

        // Legacy — kept for shooter trig (distance-to-target estimation)
        public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(80);
    }

    public static final class ShooterConstants {
        // CAN IDs
        public static final int FLYWHEEL_ID = 59;
        public static final int FLYWHEEL_FOLLOWER_ID = 60; // second SparkFlex, update to match wiring
        public static final boolean FLYWHEEL_FOLLOWER_INVERTED = false; // flip if wired opposite to leader
        public static final int HOOD_ID = 58;
        public static final int TURRET_ID = 41;
        public static final int LOADER_ID = 54;

        // Flywheel
        public static final double FLYWHEEL_GEARING = 1.0;
        public static final double FLYWHEEL_MOI = 0.005; // kg * m^2
        public static final double FLYWHEEL_TOLERANCE_RPM = 50.0;
        public static final double FLYWHEEL_MAX_RPM = 6000.0;
        public static final double FLYWHEEL_CURRENT_LIMIT_AMPS = 60.0;
        public static final double FLYWHEEL_VOLTAGE_COMP_VOLTS = 12.0;
        public static final double FLYWHEEL_KP = 0.0001;
        public static final double FLYWHEEL_KI = 0.0;
        public static final double FLYWHEEL_KD = 0.0;
        public static final double FLYWHEEL_KS = 0.01;
        public static final double FLYWHEEL_KV = 0.002;

        // Hood
        public static final double HOOD_GEARING = 20.0; // 20:1
        public static final double HOOD_MIN_ANGLE_DEG = 10.0;
        public static final double HOOD_MAX_ANGLE_DEG = 60.0;
        public static final double HOOD_TOLERANCE_DEG = 1.0;
        public static final double HOOD_OFFSET_DEG = 0.0; // Calibration offset
        public static final double HOOD_HOME_ANGLE_DEG = 10.0;
        public static final double HOOD_MOI = 0.01; // kg * m^2
        public static final double HOOD_CURRENT_LIMIT_AMPS = 20.0;
        public static final double HOOD_KP = 2.0;
        public static final double HOOD_KI = 0.0;
        public static final double HOOD_KD = 0.0;
        public static final double HOOD_MIN_HARD_LIMIT_DEG = 0.0;
        public static final double HOOD_MAX_HARD_LIMIT_DEG = 200.0;

        // Turret
        public static final double TURRET_GEARING = 200 / 19.0; // Example 50:1
        // CRT absolute encoders (REV Through Bore, DIO ports - update to match your wiring)
        public static final int TURRET_ENC1_DIO_PORT = 0; // 19T pinion on 200T ring
        public static final int TURRET_ENC2_DIO_PORT = 1; // 21T pinion on 200T ring
        // Encoder rotations per one turret rotation
        public static final double TURRET_ENC1_RATIO = 200.0 / 19.0;
        public static final double TURRET_ENC2_RATIO = 200.0 / 21.0;
        public static final double TURRET_CRT_TOLERANCE_ROT = 1e-3;
        public static final double TURRET_MIN_ANGLE_DEG = -170.0;
        public static final double TURRET_MAX_ANGLE_DEG = 170.0;
        public static final double TURRET_MIN_HARD_LIMIT_DEG = -180.0;
        public static final double TURRET_MAX_HARD_LIMIT_DEG = 180.0;
        public static final double TURRET_TOLERANCE_DEG = 1.0;
        public static final double TURRET_OFFSET_DEG = 0.0; // Calibration offset
        public static final double TURRET_HOME_ANGLE_DEG = 0.0;
        public static final double TURRET_MOI = 0.1; // kg * m^2
        public static final double TURRET_CURRENT_LIMIT_AMPS = 30.0;
        public static final double TURRET_KP = 4.0;
        public static final double TURRET_KI = 0.0;
        public static final double TURRET_KD = 0.0;
        // NOTE: Turret is CABLE WRAP limited, so NO continuous wrapping.

        // Loader
        public static final double LOADER_FEED_SPEED = 0.8;
        public static final int BEAM_BREAK_CHANNEL = 0;
        public static final double LOADER_CURRENT_LIMIT_AMPS = 20.0;
        public static final double LOADER_GEARING = 1.0;
        public static final double LOADER_KP = 0.001;
        public static final double LOADER_KI = 0.0;
        public static final double LOADER_KD = 0.0;

    }

    public static final class IntakeConstants {
        // Roller — SparkMax + NEO 550 (open loop)
        public static final int INTAKE_MOTOR_ID = 20;
        public static final double INTAKE_IN_SPEED = 0.8;
        public static final double INTAKE_OUT_SPEED = -0.5;
        public static final double INTAKE_CURRENT_LIMIT_AMPS = 20.0;
        public static final double INTAKE_GEARING = 1.0;

        // Pivot — TalonSRX + integrated quadrature encoder (4096 ticks/rev), closed loop
        public static final int INTAKE_PIVOT_ID = 23;
        public static final double INTAKE_PIVOT_GEARING = 20.0;      // update to match hardware
        public static final double INTAKE_PIVOT_DEPLOYED_DEG = 90.0; // angle when intake is down
        public static final double INTAKE_PIVOT_STOWED_DEG = 0.0;    // angle when intake is up
        public static final double INTAKE_PIVOT_MIN_HARD_DEG = -5.0;
        public static final double INTAKE_PIVOT_MAX_HARD_DEG = 100.0;
        public static final double INTAKE_PIVOT_TOLERANCE_DEG = 2.0;
        public static final double INTAKE_PIVOT_CURRENT_LIMIT_AMPS = 40.0;
        // PID in TalonSRX sensor units (ticks). Tune on robot.
        public static final double INTAKE_PIVOT_KP = 0.5;
        public static final double INTAKE_PIVOT_KI = 0.0;
        public static final double INTAKE_PIVOT_KD = 0.0;
    }

    public static final class ClimberConstants {
        public static final int CLIMBER_MOTOR_ID = 21;        // SparkFlex + NeoVortex

        // Gearing & spool
        public static final double CLIMBER_GEARING = 81.0;
        public static final double CLIMBER_SPOOL_DIAMETER_INCHES = 0.65;

        // Height limits (meters)
        public static final double CLIMBER_MIN_HEIGHT_METERS = 0.0;
        public static final double CLIMBER_MAX_HEIGHT_METERS = 0.5;   // full extension
        public static final double CLIMBER_TOLERANCE_METERS = 0.01;

        // Current & mode
        public static final double CLIMBER_CURRENT_LIMIT_AMPS = 60.0;

        // PID / FF — tune after assembly
        public static final double CLIMBER_KP = 4.0;
        public static final double CLIMBER_KI = 0.0;
        public static final double CLIMBER_KD = 0.0;
        public static final double CLIMBER_KS = 0.0;
        public static final double CLIMBER_KG = 0.0;  // gravity feedforward (V)
        public static final double CLIMBER_KV = 0.0;

        // Carriage mass for sim (kg)
        public static final double CLIMBER_CARRIAGE_MASS_KG = 2.0;
    }
}
