package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeSubsystem extends SubsystemBase {

    // Kept for dashboard continuity with the previous Talon-based implementation.
    private static final int ENCODER_TICKS_PER_REV = 4096;
    private static final double TICKS_PER_DEGREE =
            ENCODER_TICKS_PER_REV * IntakeConstants.INTAKE_PIVOT_GEARING / 360.0;

    // Roller — SparkMax + NEO 550, open loop
    private final SparkMax roller;

    // Intake arm — SparkMax + NEO, closed-loop position
    private final SmartMotorController pivotMotor;
    private final Arm pivot;

    private double targetDeg = IntakeConstants.INTAKE_PIVOT_STOWED_DEG;
    private double commandedDeg = IntakeConstants.INTAKE_PIVOT_STOWED_DEG;

    public IntakeSubsystem() {
        roller = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit((int) IntakeConstants.INTAKE_CURRENT_LIMIT_AMPS)
                .inverted(false);
        roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMax pivotSpark = new SparkMax(IntakeConstants.INTAKE_PIVOT_ID, MotorType.kBrushless);
        SmartMotorControllerConfig pivotMotorConfig = new SmartMotorControllerConfig(this)
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withIdleMode(MotorMode.BRAKE)
                .withStatorCurrentLimit(Amps.of(IntakeConstants.INTAKE_PIVOT_CURRENT_LIMIT_AMPS))
                .withGearing(new MechanismGearing(IntakeConstants.INTAKE_PIVOT_GEARING))
                .withStartingPosition(Degrees.of(IntakeConstants.INTAKE_PIVOT_STOWED_DEG))
                .withClosedLoopController(
                        IntakeConstants.INTAKE_PIVOT_KP,
                        IntakeConstants.INTAKE_PIVOT_KI,
                        IntakeConstants.INTAKE_PIVOT_KD,
                        DegreesPerSecond.of(IntakeConstants.INTAKE_PIVOT_MAX_VELOCITY_DEG_PER_SEC),
                        DegreesPerSecondPerSecond.of(IntakeConstants.INTAKE_PIVOT_MAX_ACCEL_DEG_PER_SEC_SQ))
                .withMotorInverted(true)
                .withTelemetry("IntakePivotMotor", TelemetryVerbosity.LOW);

        pivotMotor = new SparkWrapper(pivotSpark, DCMotor.getNEO(1), pivotMotorConfig);

        ArmConfig pivotConfig = new ArmConfig(pivotMotor)
                .withStartingPosition(Degrees.of(IntakeConstants.INTAKE_PIVOT_STOWED_DEG))
                .withSoftLimits(
                        Degrees.of(IntakeConstants.INTAKE_PIVOT_MIN_HARD_DEG),
                        Degrees.of(IntakeConstants.INTAKE_PIVOT_MAX_HARD_DEG))
                .withLength(Meters.of(IntakeConstants.INTAKE_PIVOT_LENGTH_METERS))
                .withMass(Kilograms.of(IntakeConstants.INTAKE_PIVOT_MASS_KG))
                .withHardLimit(
                        Degrees.of(IntakeConstants.INTAKE_PIVOT_MIN_HARD_DEG),
                        Degrees.of(IntakeConstants.INTAKE_PIVOT_MAX_HARD_DEG))
                .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

        pivot = new Arm(pivotConfig);
        applyPivotTarget();
    }

    // -----------------------------------------------------------------------
    // Roller
    // -----------------------------------------------------------------------

    public void intake()     { roller.set(IntakeConstants.INTAKE_IN_SPEED); }
    public void outtake()    { roller.set(IntakeConstants.INTAKE_OUT_SPEED); }
    public void stopRoller() { roller.set(0); }

    // -----------------------------------------------------------------------
    // Pivot
    // -----------------------------------------------------------------------

    public void setAngleDeg(double deg) {
        double clamped = Math.max(
                IntakeConstants.INTAKE_PIVOT_MIN_HARD_DEG,
                Math.min(IntakeConstants.INTAKE_PIVOT_MAX_HARD_DEG, deg));
        targetDeg = clamped;
        applyPivotTarget();
    }

    public double getPivotAngleDeg() {
        return pivot.getAngle().in(Degrees);
    }

    public boolean pivotAtSetpoint() {
        return Math.abs(getPivotAngleDeg() - targetDeg) <= IntakeConstants.INTAKE_PIVOT_TOLERANCE_DEG;
    }

    public void runPivotOpenLoop(double dutyCycle) {
        commandedDeg = getPivotAngleDeg();
        pivotMotor.stopClosedLoopController();
        pivotMotor.setDutyCycle(dutyCycle);
    }

    public void holdCurrentPivotPosition() {
        targetDeg = getPivotAngleDeg();
        applyPivotTarget();
    }

    // -----------------------------------------------------------------------
    // Commands
    // -----------------------------------------------------------------------

    public Command deployCommand() {
        return Commands.runOnce(() -> setAngleDeg(IntakeConstants.INTAKE_PIVOT_DEPLOYED_DEG), this)
                .andThen(Commands.waitUntil(this::pivotAtSetpoint));
    }

    public Command stowCommand() {
        return Commands.runOnce(() -> setAngleDeg(IntakeConstants.INTAKE_PIVOT_STOWED_DEG), this)
                .andThen(Commands.waitUntil(this::pivotAtSetpoint));
    }

    public Command intakeSequence() {
        return Commands.sequence(
                deployCommand(),
                Commands.run(this::intake, this));
    }

    public Command stowSequence() {
        return Commands.sequence(
                Commands.runOnce(this::stopRoller, this),
                stowCommand());
    }

    // -----------------------------------------------------------------------
    // Periodic
    // -----------------------------------------------------------------------

    @Override
    public void periodic() {
        pivot.updateTelemetry();
        SmartDashboard.putNumber("Intake/PivotAngleDeg", getPivotAngleDeg());
        SmartDashboard.putNumber("Intake/PivotRawTicks", degreesToTicks(getPivotAngleDeg()));
        SmartDashboard.putNumber("Intake/TargetAngleDeg", targetDeg);
        SmartDashboard.putNumber("Intake/CommandedAngleDeg", commandedDeg);
        SmartDashboard.putNumber("Intake/PivotErrorDeg", targetDeg - getPivotAngleDeg());
        SmartDashboard.putBoolean("Intake/PivotAtSetpoint", pivotAtSetpoint());
        SmartDashboard.putNumber("Intake/PivotOutputPercent", pivotMotor.getDutyCycle());
        SmartDashboard.putNumber("Intake/PivotCurrentAmps", pivotMotor.getStatorCurrent().in(Amps));
        SmartDashboard.putNumber("Intake/RollerDutyCycle", roller.get());
    }

    @Override
    public void simulationPeriodic() {
        pivot.simIterate();
    }

    private void applyPivotTarget() {
        commandedDeg = targetDeg;
        if (isStowHoldActive()) {
            commandedDeg = Math.min(
                    IntakeConstants.INTAKE_PIVOT_MAX_HARD_DEG,
                    targetDeg + IntakeConstants.INTAKE_PIVOT_STOW_HOLD_BIAS_DEG);
        }
        pivot.setMechanismPositionSetpoint(Degrees.of(commandedDeg));
    }

    private boolean isStowHoldActive() {
        return Math.abs(targetDeg - IntakeConstants.INTAKE_PIVOT_STOWED_DEG)
                <= IntakeConstants.INTAKE_PIVOT_TOLERANCE_DEG;
    }

    private static double degreesToTicks(double deg) {
        return deg * TICKS_PER_DEGREE;
    }
}
