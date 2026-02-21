package frc.robot.configs;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants.ShooterConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.PivotConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.gearing.MechanismGearing;

public class ShooterConfigFactory {

    public static SmartMotorControllerConfig createFlywheelMotorConfig(Subsystem subsystem) {
        return new SmartMotorControllerConfig(subsystem)
            .withIdleMode(MotorMode.COAST)
            .withVoltageCompensation(Units.Volts.of(ShooterConstants.FLYWHEEL_VOLTAGE_COMP_VOLTS))
            .withStatorCurrentLimit(Units.Amps.of(ShooterConstants.FLYWHEEL_CURRENT_LIMIT_AMPS))
            .withMotorInverted(false)
            .withGearing(new MechanismGearing(ShooterConstants.FLYWHEEL_GEARING))
            // PID (feedforward omitted — parameter 204 not supported on this firmware)
            .withClosedLoopController(ShooterConstants.FLYWHEEL_KP, ShooterConstants.FLYWHEEL_KI, ShooterConstants.FLYWHEEL_KD)
            .withSimClosedLoopController(ShooterConstants.FLYWHEEL_KP, ShooterConstants.FLYWHEEL_KI, ShooterConstants.FLYWHEEL_KD)
            .withSimFeedforward(new SimpleMotorFeedforward(ShooterConstants.FLYWHEEL_KS, ShooterConstants.FLYWHEEL_KV))
            .withClosedLoopTolerance(Units.Rotations.of(ShooterConstants.FLYWHEEL_TOLERANCE_RPM / 60.0)) // RPS
            .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH);
    }

    public static FlyWheelConfig createFlywheelMechanismConfig(SmartMotorController motor) {
        return new FlyWheelConfig(motor)
            .withMOI(ShooterConstants.FLYWHEEL_MOI)
            // .withMass(...) // Optional: Add mass if known for sim
            // .withDiameter(...) // Optional: Add diameter if known
            .withUpperSoftLimit(Units.RPM.of(ShooterConstants.FLYWHEEL_MAX_RPM))
            .withTelemetry("Flywheel", TelemetryVerbosity.HIGH);
    }

    public static SmartMotorControllerConfig createHoodMotorConfig(Subsystem subsystem) {
        return new SmartMotorControllerConfig(subsystem)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Units.Amps.of(ShooterConstants.HOOD_CURRENT_LIMIT_AMPS))
            .withGearing(new MechanismGearing(ShooterConstants.HOOD_GEARING))
            .withSoftLimit(Units.Degrees.of(ShooterConstants.HOOD_MIN_ANGLE_DEG), Units.Degrees.of(ShooterConstants.HOOD_MAX_ANGLE_DEG))
            .withClosedLoopController(ShooterConstants.HOOD_KP, ShooterConstants.HOOD_KI, ShooterConstants.HOOD_KD)
            .withSimClosedLoopController(ShooterConstants.HOOD_KP, ShooterConstants.HOOD_KI, ShooterConstants.HOOD_KD)
            .withClosedLoopTolerance(Units.Degrees.of(ShooterConstants.HOOD_TOLERANCE_DEG))
            .withTelemetry("HoodMotor", TelemetryVerbosity.LOW);
    }

    public static PivotConfig createHoodMechanismConfig(SmartMotorController motor) {
        return new PivotConfig(motor)
            .withHardLimit(Units.Degrees.of(ShooterConstants.HOOD_MIN_HARD_LIMIT_DEG), Units.Degrees.of(ShooterConstants.HOOD_MAX_HARD_LIMIT_DEG))
            .withStartingPosition(Units.Degrees.of(ShooterConstants.HOOD_HOME_ANGLE_DEG))
            .withMOI(ShooterConstants.HOOD_MOI)
            .withTelemetry("Hood", TelemetryVerbosity.HIGH);
    }

    public static SmartMotorControllerConfig createTurretMotorConfig(Subsystem subsystem) {
        return new SmartMotorControllerConfig(subsystem)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Units.Amps.of(ShooterConstants.TURRET_CURRENT_LIMIT_AMPS))
            .withGearing(new MechanismGearing(ShooterConstants.TURRET_GEARING))
            // CABLE WRAP LIMITS (Soft Limits)
            .withSoftLimit(Units.Degrees.of(ShooterConstants.TURRET_MIN_ANGLE_DEG), Units.Degrees.of(ShooterConstants.TURRET_MAX_ANGLE_DEG))
            .withClosedLoopController(ShooterConstants.TURRET_KP, ShooterConstants.TURRET_KI, ShooterConstants.TURRET_KD)
            .withSimClosedLoopController(ShooterConstants.TURRET_KP, ShooterConstants.TURRET_KI, ShooterConstants.TURRET_KD)
            .withClosedLoopTolerance(Units.Degrees.of(ShooterConstants.TURRET_TOLERANCE_DEG))
            .withTelemetry("TurretMotor", TelemetryVerbosity.LOW);
    }

    public static PivotConfig createTurretMechanismConfig(SmartMotorController motor) {
        return new PivotConfig(motor)
            .withStartingPosition(Units.Degrees.of(ShooterConstants.TURRET_HOME_ANGLE_DEG))
            .withMOI(ShooterConstants.TURRET_MOI)
            .withHardLimit(Units.Degrees.of(ShooterConstants.TURRET_MIN_HARD_LIMIT_DEG), Units.Degrees.of(ShooterConstants.TURRET_MAX_HARD_LIMIT_DEG))
            .withTelemetry("Turret", TelemetryVerbosity.HIGH);
    }

    public static SmartMotorControllerConfig createLoaderConfig(Subsystem subsystem) {
        return new SmartMotorControllerConfig(subsystem)
            .withControlMode(ControlMode.OPEN_LOOP)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Units.Amps.of(ShooterConstants.LOADER_CURRENT_LIMIT_AMPS))
            .withGearing(new MechanismGearing(ShooterConstants.LOADER_GEARING))
            .withMotorInverted(false)
           // .withClosedLoopController(ShooterConstants.LOADER_KP, ShooterConstants.LOADER_KI, ShooterConstants.LOADER_KD)
            .withTelemetry("LoaderMotor", TelemetryVerbosity.LOW);
    }
}
