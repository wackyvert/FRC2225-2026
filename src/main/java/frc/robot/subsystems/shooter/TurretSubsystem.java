package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.ShooterConfigFactory;
import frc.robot.constants.Constants.ShooterConstants;

import static edu.wpi.first.units.Units.Degrees;

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
    private String crtStatus = "NOT_ATTEMPTED";
    private double crtErrorRot = Double.NaN;
    private double crtSeedAngleDeg = Double.NaN;
    private double crtEncoder1SpreadRot = Double.NaN;
    private double crtEncoder2SpreadRot = Double.NaN;
    private int crtIterations = 0;

    // REV Through Bore encoders on DIO — read ONCE at startup to seed the relative encoder
    private final DutyCycleEncoder encoder1 = new DutyCycleEncoder(ShooterConstants.TURRET_ENC1_DIO_PORT);
    private final DutyCycleEncoder encoder2 = new DutyCycleEncoder(ShooterConstants.TURRET_ENC2_DIO_PORT);

    public TurretSubsystem() {
        SparkFlex spark = new SparkFlex(ShooterConstants.TURRET_ID, MotorType.kBrushless);

        SmartMotorControllerConfig motorConfig = ShooterConfigFactory.createTurretMotorConfig(this);
        motor = new SparkWrapper(spark, DCMotor.getNEO(1), motorConfig);

        PivotConfig mechConfig = ShooterConfigFactory.createTurretMechanismConfig(motor);
        pivot = new Pivot(mechConfig);

        if (RobotBase.isReal()) {
            seedEncoderFromCRT();
        } else {
            motor.setEncoderPosition(Units.Rotations.of(0));
        }

        SmartMotorControllerCommandRegistry.addCommand("Turret Center", this, () -> pivot.setAngle(Degrees.of(0)));
    }

    /**
     * Solves the turret's absolute position via CRT using two absolute encoders,
     * then seeds the motor's relative encoder so closed-loop control is correct
     * from boot without requiring a homing sequence.
     *
     * Must be called once during construction, while the turret is stationary.
     */
    private void seedEncoderFromCRT() {
        Optional<CrtSample> startupSample = captureStableCrtSample();
        if (startupSample.isEmpty()) {
            crtStatus = "UNSTABLE_STARTUP";
            crtSeedAngleDeg = ShooterConstants.TURRET_OFFSET_DEG;
            motor.setEncoderPosition(Units.Degrees.of(ShooterConstants.TURRET_OFFSET_DEG));
            return;
        }

        CrtSample sample = startupSample.get();
        EasyCRTConfig crtConfig = new EasyCRTConfig(
                () -> Units.Rotations.of(sample.encoder1Rot()),
                () -> Units.Rotations.of(sample.encoder2Rot()))
                .withEncoderRatios(ShooterConstants.TURRET_ENC1_RATIO, ShooterConstants.TURRET_ENC2_RATIO)
                .withAbsoluteEncoderInversions(
                        ShooterConstants.TURRET_ENC1_INVERTED,
                        ShooterConstants.TURRET_ENC2_INVERTED)
                .withAbsoluteEncoderOffsets(
                        Units.Rotations.of(ShooterConstants.TURRET_ENC1_OFFSET_ROT),
                        Units.Rotations.of(ShooterConstants.TURRET_ENC2_OFFSET_ROT))
                // Use hard limits as the mechanism range so we have a small margin beyond the soft limits.
                // Per YAMS docs, keep this range negative-possible to avoid ambiguity at boundaries.
                .withMechanismRange(
                        Units.Rotations.of(ShooterConstants.TURRET_MIN_HARD_LIMIT_DEG / 360.0),
                        Units.Rotations.of(ShooterConstants.TURRET_MAX_HARD_LIMIT_DEG / 360.0))
                .withMatchTolerance(Units.Rotations.of(ShooterConstants.TURRET_CRT_TOLERANCE_ROT));

        EasyCRT crt = new EasyCRT(crtConfig);
        Optional<edu.wpi.first.units.measure.Angle> solved = crt.getAngleOptional();
        crtIterations = crt.getLastIterations();

        if (solved.isPresent()) {
            var seededAngle = solved.get().plus(Units.Degrees.of(ShooterConstants.TURRET_OFFSET_DEG));
            motor.setEncoderPosition(seededAngle);
            crtStatus = crt.getLastStatus().name();
            crtErrorRot = crt.getLastErrorRotations();
            crtSeedAngleDeg = seededAngle.in(Units.Degrees);
        } else {
            crtStatus = crt.getLastStatus().name();
            crtErrorRot = crt.getLastErrorRotations();
            crtSeedAngleDeg = ShooterConstants.TURRET_OFFSET_DEG;
            DriverStation.reportWarning(
                    "[TurretSubsystem] CRT failed to resolve turret position: " + crt.getLastStatus()
                    + " (error=" + crt.getLastErrorRotations() + " rot). "
                    + "Defaulting to 0. Check encoder wiring and DIO ports.",
                    false);
            motor.setEncoderPosition(Units.Degrees.of(ShooterConstants.TURRET_OFFSET_DEG));
        }
    }

    private Optional<CrtSample> captureStableCrtSample() {
        if (!encoder1.isConnected() || !encoder2.isConnected()) {
            DriverStation.reportWarning("[TurretSubsystem] CRT encoder not connected at startup.", false);
            return Optional.empty();
        }

        Timer.delay(ShooterConstants.TURRET_CRT_STARTUP_SETTLE_TIME_S);

        double[] encoder1Samples = new double[ShooterConstants.TURRET_CRT_SAMPLE_COUNT];
        double[] encoder2Samples = new double[ShooterConstants.TURRET_CRT_SAMPLE_COUNT];

        for (int i = 0; i < ShooterConstants.TURRET_CRT_SAMPLE_COUNT; i++) {
            encoder1Samples[i] = encoder1.get();
            encoder2Samples[i] = encoder2.get();
            if (i + 1 < ShooterConstants.TURRET_CRT_SAMPLE_COUNT) {
                Timer.delay(ShooterConstants.TURRET_CRT_SAMPLE_PERIOD_S);
            }
        }

        double encoder1Spread = getWrappedSpread(encoder1Samples);
        double encoder2Spread = getWrappedSpread(encoder2Samples);
        crtEncoder1SpreadRot = encoder1Spread;
        crtEncoder2SpreadRot = encoder2Spread;

        if (encoder1Spread > ShooterConstants.TURRET_CRT_MAX_SAMPLE_SPREAD_ROT
                || encoder2Spread > ShooterConstants.TURRET_CRT_MAX_SAMPLE_SPREAD_ROT) {
            DriverStation.reportWarning(
                    "[TurretSubsystem] CRT startup samples were unstable. "
                    + "Encoder1 spread=" + encoder1Spread + " rot, Encoder2 spread=" + encoder2Spread + " rot.",
                    false);
            return Optional.empty();
        }

        return Optional.of(new CrtSample(
                getWrappedAverage(encoder1Samples),
                getWrappedAverage(encoder2Samples)));
    }

    private double getWrappedAverage(double[] samples) {
        double sinSum = 0.0;
        double cosSum = 0.0;
        for (double sample : samples) {
            double angleRad = sample * 2.0 * Math.PI;
            sinSum += Math.sin(angleRad);
            cosSum += Math.cos(angleRad);
        }

        double averageRot = Math.atan2(sinSum / samples.length, cosSum / samples.length) / (2.0 * Math.PI);
        if (averageRot < 0.0) {
            averageRot += 1.0;
        }
        return averageRot;
    }

    private double getWrappedSpread(double[] samples) {
        double average = getWrappedAverage(samples);
        double maxError = 0.0;
        for (double sample : samples) {
            double error = Math.abs(Math.IEEEremainder(sample - average, 1.0));
            maxError = Math.max(maxError, error);
        }
        return maxError;
    }

    private record CrtSample(double encoder1Rot, double encoder2Rot) {}

    public Command setAngleDegF(double degrees) {
        return pivot.setAngle(Units.Degrees.of(degrees));
    }

    public void setAngleDegSetpoint(double degrees) {
        targetAngle = degrees;
        pivot.setMechanismPositionSetpoint(Units.Degrees.of(degrees));
    }

    public Command jogDeg(double deltaDeg) {
        return runOnce(() -> pivot.setAngle(Units.Degrees.of(getAngleDeg() + deltaDeg)).schedule());
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
        SmartDashboard.putNumber("Turret/ErrorDeg", targetAngle - getAngleDeg());
        SmartDashboard.putBoolean("Turret/AtSetpoint", atSetpoint());
        SmartDashboard.putNumber("Turret/Encoder1Raw", encoder1.get());
        SmartDashboard.putNumber("Turret/Encoder2Raw", encoder2.get());
        SmartDashboard.putNumber(
                "Turret/Encoder1Adjusted",
                MathUtil.inputModulus(
                        encoder1.get() + ShooterConstants.TURRET_ENC1_OFFSET_ROT,
                        0.0,
                        1.0));
        SmartDashboard.putNumber(
                "Turret/Encoder2Adjusted",
                MathUtil.inputModulus(
                        encoder2.get() + ShooterConstants.TURRET_ENC2_OFFSET_ROT,
                        0.0,
                        1.0));
        SmartDashboard.putString("Turret/CRTStatus", crtStatus);
        SmartDashboard.putNumber("Turret/CRTErrorRot", crtErrorRot);
        SmartDashboard.putNumber("Turret/CRTSeedAngleDeg", crtSeedAngleDeg);
        SmartDashboard.putNumber("Turret/CRTEncoder1SpreadRot", crtEncoder1SpreadRot);
        SmartDashboard.putNumber("Turret/CRTEncoder2SpreadRot", crtEncoder2SpreadRot);
        SmartDashboard.putNumber("Turret/CRTIterations", crtIterations);
    }

    @Override
    public void simulationPeriodic() {
        pivot.simIterate();
    }
}
