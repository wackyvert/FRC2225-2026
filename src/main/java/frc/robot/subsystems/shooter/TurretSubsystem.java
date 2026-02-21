package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.ShooterConfigFactory;
import frc.robot.constants.Constants.ShooterConstants;
import java.util.Optional;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerCommandRegistry;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretSubsystem extends SubsystemBase {

    private final SmartMotorController motor;
    private final Pivot pivot;
    private double targetAngle = 0;

    // REV Through Bore encoders on DIO — read ONCE at startup to seed the relative encoder
    private final DutyCycleEncoder encoder1 = new DutyCycleEncoder(ShooterConstants.TURRET_ENC1_DIO_PORT);
    private final DutyCycleEncoder encoder2 = new DutyCycleEncoder(ShooterConstants.TURRET_ENC2_DIO_PORT);

    public TurretSubsystem() {
        SparkMax spark = new SparkMax(ShooterConstants.TURRET_ID, MotorType.kBrushless);

        SmartMotorControllerConfig motorConfig = ShooterConfigFactory.createTurretMotorConfig(this);
        motor = new SparkWrapper(spark, DCMotor.getNEO(1), motorConfig);

        PivotConfig mechConfig = ShooterConfigFactory.createTurretMechanismConfig(motor);
        pivot = new Pivot(mechConfig);

        seedEncoderFromCRT();

        SmartMotorControllerCommandRegistry.addCommand("Turret Center", this, () -> setAngleDeg(0));
    }

    /**
     * Solves the turret's absolute position via CRT using two absolute encoders,
     * then seeds the motor's relative encoder so closed-loop control is correct
     * from boot without requiring a homing sequence.
     *
     * Must be called once during construction, while the turret is stationary.
     */
    private void seedEncoderFromCRT() {
        EasyCRTConfig crtConfig = new EasyCRTConfig(
                // DutyCycleEncoder.get() already returns [0, 1); EasyCRT wraps internally anyway
                () -> Units.Rotations.of(encoder1.get()),
                () -> Units.Rotations.of(encoder2.get()))
                .withEncoderRatios(ShooterConstants.TURRET_ENC1_RATIO, ShooterConstants.TURRET_ENC2_RATIO)
                // Use hard limits as the mechanism range so we have a small margin beyond the soft limits.
                // Per YAMS docs, keep this range negative-possible to avoid ambiguity at boundaries.
                .withMechanismRange(
                        Units.Rotations.of(ShooterConstants.TURRET_MIN_HARD_LIMIT_DEG / 360.0),
                        Units.Rotations.of(ShooterConstants.TURRET_MAX_HARD_LIMIT_DEG / 360.0))
                .withMatchTolerance(Units.Rotations.of(ShooterConstants.TURRET_CRT_TOLERANCE_ROT));

        EasyCRT crt = new EasyCRT(crtConfig);
        Optional<edu.wpi.first.units.measure.Angle> solved = crt.getAngleOptional();

        if (solved.isPresent()) {
            motor.setEncoderPosition(solved.get());
        } else {
            DriverStation.reportWarning(
                    "[TurretSubsystem] CRT failed to resolve turret position: " + crt.getLastStatus()
                    + " (error=" + crt.getLastErrorRotations() + " rot). "
                    + "Defaulting to 0. Check encoder wiring and DIO ports.",
                    false);
            motor.setEncoderPosition(Units.Rotations.of(0));
        }
    }

    public void setAngleDeg(double degrees) {
        double clamped = Math.max(ShooterConstants.TURRET_MIN_ANGLE_DEG,
                Math.min(ShooterConstants.TURRET_MAX_ANGLE_DEG, degrees));
        targetAngle = clamped;
        pivot.setAngle(Units.Degrees.of(clamped));
    }

    public double getAngleDeg() {
        return pivot.getAngle().in(Units.Degrees);
    }

    public boolean atSetpoint() {
        return pivot.isNear(Units.Degrees.of(targetAngle),
                Units.Degrees.of(ShooterConstants.TURRET_TOLERANCE_DEG)).getAsBoolean();
    }

    @Override
    public void periodic() {
        pivot.updateTelemetry();
        SmartDashboard.putNumber("Turret/AngleDeg", getAngleDeg());
        SmartDashboard.putNumber("Turret/TargetAngleDeg", targetAngle);
        SmartDashboard.putBoolean("Turret/AtSetpoint", atSetpoint());
        SmartDashboard.putNumber("Turret/Encoder1Raw", encoder1.get());
        SmartDashboard.putNumber("Turret/Encoder2Raw", encoder2.get());
    }

    @Override
    public void simulationPeriodic() {
        pivot.simIterate();
    }
}
