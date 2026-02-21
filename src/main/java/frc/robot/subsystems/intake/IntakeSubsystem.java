package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeSubsystem extends SubsystemBase {

    // Roller — SparkMax + NEO 550, open-loop duty cycle
    private final SmartMotorController rollerMotor;

    // Pivot — SparkFlex + NeoVortex, closed-loop position
    private final SmartMotorController pivotMotor;
    private final Pivot pivot;

    public IntakeSubsystem() {
        // --- Roller ---
        SparkMax rollerSpark = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        SmartMotorControllerConfig rollerConfig = new SmartMotorControllerConfig(this)
                .withControlMode(ControlMode.OPEN_LOOP)
                .withIdleMode(MotorMode.COAST)
                .withStatorCurrentLimit(Units.Amps.of(IntakeConstants.INTAKE_CURRENT_LIMIT_AMPS))
                .withGearing(new MechanismGearing(IntakeConstants.INTAKE_GEARING))
                .withMotorInverted(false)
                .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.LOW);
        rollerMotor = new SparkWrapper(rollerSpark, DCMotor.getNeo550(1), rollerConfig);

        // --- Pivot ---
        SparkFlex pivotSpark = new SparkFlex(IntakeConstants.INTAKE_PIVOT_ID, MotorType.kBrushless);
        SmartMotorControllerConfig pivotConfig = new SmartMotorControllerConfig(this)
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withIdleMode(MotorMode.BRAKE)
                .withStatorCurrentLimit(Units.Amps.of(IntakeConstants.INTAKE_PIVOT_CURRENT_LIMIT_AMPS))
                .withGearing(new MechanismGearing(IntakeConstants.INTAKE_PIVOT_GEARING))
                .withSoftLimit(
                        Units.Degrees.of(IntakeConstants.INTAKE_PIVOT_STOWED_DEG),
                        Units.Degrees.of(IntakeConstants.INTAKE_PIVOT_DEPLOYED_DEG))
                .withClosedLoopController(
                        IntakeConstants.INTAKE_PIVOT_KP,
                        IntakeConstants.INTAKE_PIVOT_KI,
                        IntakeConstants.INTAKE_PIVOT_KD)
                .withClosedLoopTolerance(Units.Degrees.of(IntakeConstants.INTAKE_PIVOT_TOLERANCE_DEG))
                .withMotorInverted(false)
                .withTelemetry("IntakePivotMotor", TelemetryVerbosity.LOW);
        pivotMotor = new SparkWrapper(pivotSpark, DCMotor.getNeoVortex(1), pivotConfig);

        PivotConfig pivotMechConfig = new PivotConfig(pivotMotor)
                .withStartingPosition(Units.Degrees.of(IntakeConstants.INTAKE_PIVOT_STOWED_DEG))
                .withHardLimit(
                        Units.Degrees.of(IntakeConstants.INTAKE_PIVOT_MIN_HARD_DEG),
                        Units.Degrees.of(IntakeConstants.INTAKE_PIVOT_MAX_HARD_DEG))
                .withMOI(IntakeConstants.INTAKE_PIVOT_MOI)
                .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);
        pivot = new Pivot(pivotMechConfig);
    }

    // -----------------------------------------------------------------------
    // Roller
    // -----------------------------------------------------------------------

    public void intake() {
        rollerMotor.setDutyCycle(IntakeConstants.INTAKE_IN_SPEED);
    }

    public void outtake() {
        rollerMotor.setDutyCycle(IntakeConstants.INTAKE_OUT_SPEED);
    }

    public void stopRoller() {
        rollerMotor.setDutyCycle(0);
    }

    // -----------------------------------------------------------------------
    // Pivot
    // -----------------------------------------------------------------------

    /** Lower the intake arm to the deployed (floor) position. */
    public Command deployCommand() {
        return pivot.setAngle(Units.Degrees.of(IntakeConstants.INTAKE_PIVOT_DEPLOYED_DEG));
    }

    /** Raise the intake arm to the stowed position. */
    public Command stowCommand() {
        return pivot.setAngle(Units.Degrees.of(IntakeConstants.INTAKE_PIVOT_STOWED_DEG));
    }

    public double getPivotAngleDeg() {
        return pivot.getAngle().in(Units.Degrees);
    }

    public boolean pivotAtSetpoint() {
        return pivot.isNear(pivot.getAngle(),
                Units.Degrees.of(IntakeConstants.INTAKE_PIVOT_TOLERANCE_DEG)).getAsBoolean();
    }

    // -----------------------------------------------------------------------
    // Compound commands
    // -----------------------------------------------------------------------

    /** Deploy then spin rollers — the typical intake sequence. */
    public Command intakeSequence() {
        return Commands.sequence(
                deployCommand(),
                Commands.run(this::intake, this));
    }

    /** Stop rollers and stow the arm. */
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
        rollerMotor.updateTelemetry();
        pivot.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        rollerMotor.simIterate();
        pivot.simIterate();
    }
}
