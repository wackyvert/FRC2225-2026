package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private static final int ENCODER_TICKS_PER_REV = 4096;
    private static final double TICKS_PER_DEGREE =
            ENCODER_TICKS_PER_REV * IntakeConstants.INTAKE_PIVOT_GEARING / 360.0;

    private final SparkMax roller;
    private final TalonFX pivotMotor;
    private final DutyCycleOut pivotDutyRequest = new DutyCycleOut(0.0);
    private final PositionDutyCycle pivotPositionRequest = new PositionDutyCycle(0.0);

    private double targetDeg = IntakeConstants.INTAKE_PIVOT_STOWED_DEG;
    private double commandedDeg = IntakeConstants.INTAKE_PIVOT_STOWED_DEG;
    private boolean openLoopActive = false;

    public IntakeSubsystem() {
        roller = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit((int) IntakeConstants.INTAKE_CURRENT_LIMIT_AMPS)
                .inverted(false);
        roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotMotor = new TalonFX(IntakeConstants.INTAKE_PIVOT_ID);
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.Slot0 = new Slot0Configs()
                .withKP(IntakeConstants.INTAKE_PIVOT_KP)
                .withKI(IntakeConstants.INTAKE_PIVOT_KI)
                .withKD(IntakeConstants.INTAKE_PIVOT_KD);
        pivotConfig.MotorOutput = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);
        pivotConfig.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IntakeConstants.INTAKE_PIVOT_CURRENT_LIMIT_AMPS)
                .withStatorCurrentLimitEnable(true);
        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setPosition(degreesToMotorRotations(IntakeConstants.INTAKE_PIVOT_STOWED_DEG));

        applyPivotTarget();
    }

    public void intake() {
        roller.set(IntakeConstants.INTAKE_IN_SPEED);
    }

    public void outtake() {
        roller.set(IntakeConstants.INTAKE_OUT_SPEED);
    }

    public void stopRoller() {
        roller.set(0.0);
    }

    public void setAngleDeg(double deg) {
        double clamped = Math.max(
                IntakeConstants.INTAKE_PIVOT_MIN_HARD_DEG,
                Math.min(IntakeConstants.INTAKE_PIVOT_MAX_HARD_DEG, deg));
        targetDeg = clamped;
        applyPivotTarget();
    }

    public double getPivotAngleDeg() {
        return motorRotationsToDegrees(pivotMotor.getPosition().getValueAsDouble());
    }

    public boolean pivotAtSetpoint() {
        return Math.abs(getPivotAngleDeg() - targetDeg) <= IntakeConstants.INTAKE_PIVOT_TOLERANCE_DEG;
    }

    public void runPivotOpenLoop(double dutyCycle) {
        openLoopActive = true;
        commandedDeg = getPivotAngleDeg();
        pivotMotor.setControl(pivotDutyRequest.withOutput(dutyCycle));
    }

    public void holdCurrentPivotPosition() {
        targetDeg = getPivotAngleDeg();
        applyPivotTarget();
    }

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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/PivotAngleDeg", getPivotAngleDeg());
        SmartDashboard.putNumber("Intake/PivotRawTicks", degreesToTicks(getPivotAngleDeg()));
        SmartDashboard.putNumber("Intake/TargetAngleDeg", targetDeg);
        SmartDashboard.putNumber("Intake/CommandedAngleDeg", commandedDeg);
        SmartDashboard.putNumber("Intake/PivotErrorDeg", targetDeg - getPivotAngleDeg());
        SmartDashboard.putBoolean("Intake/PivotAtSetpoint", pivotAtSetpoint());
        SmartDashboard.putBoolean("Intake/PivotOpenLoop", openLoopActive);
        SmartDashboard.putNumber("Intake/PivotOutputPercent", pivotMotor.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Intake/PivotCurrentAmps", pivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/RollerDutyCycle", roller.get());
    }

    private void applyPivotTarget() {
        openLoopActive = false;
        commandedDeg = targetDeg;
        if (isStowHoldActive()) {
            commandedDeg = Math.min(
                    IntakeConstants.INTAKE_PIVOT_MAX_HARD_DEG,
                    targetDeg + IntakeConstants.INTAKE_PIVOT_STOW_HOLD_BIAS_DEG);
        }
        pivotMotor.setControl(pivotPositionRequest.withPosition(degreesToMotorRotations(commandedDeg)));
    }

    private boolean isStowHoldActive() {
        return Math.abs(targetDeg - IntakeConstants.INTAKE_PIVOT_STOWED_DEG)
                <= IntakeConstants.INTAKE_PIVOT_TOLERANCE_DEG;
    }

    private static double degreesToMotorRotations(double deg) {
        return (deg / 360.0) * IntakeConstants.INTAKE_PIVOT_GEARING;
    }

    private static double motorRotationsToDegrees(double motorRotations) {
        return (motorRotations / IntakeConstants.INTAKE_PIVOT_GEARING) * 360.0;
    }

    private static double degreesToTicks(double deg) {
        return deg * TICKS_PER_DEGREE;
    }
}
