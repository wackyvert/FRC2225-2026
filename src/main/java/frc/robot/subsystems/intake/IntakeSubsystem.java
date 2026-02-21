package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

    // TalonSRX integrated quadrature encoder: 4096 ticks per motor revolution
    private static final int ENCODER_TICKS_PER_REV = 4096;
    private static final double TICKS_PER_DEGREE =
            ENCODER_TICKS_PER_REV * IntakeConstants.INTAKE_PIVOT_GEARING / 360.0;

    // Roller — SparkMax + NEO 550, open loop
    private final SparkMax roller;

    // Pivot — TalonSRX + integrated encoder, closed-loop position
    private final TalonSRX pivot;
    
    private double targetDeg = IntakeConstants.INTAKE_PIVOT_STOWED_DEG;

    public IntakeSubsystem() {
       // pivot.configSelectedFeedbackSensor(FeedbackDevice.Analog)
        // --- Roller ---
        roller = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit((int) IntakeConstants.INTAKE_CURRENT_LIMIT_AMPS)
                .inverted(false);
        roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- Pivot ---
        pivot = new TalonSRX(IntakeConstants.INTAKE_PIVOT_ID);

        TalonSRXConfiguration pivotCfg = new TalonSRXConfiguration();
        // PID slot 0
        pivotCfg.slot0.kP = IntakeConstants.INTAKE_PIVOT_KP;
        pivotCfg.slot0.kI = IntakeConstants.INTAKE_PIVOT_KI;
        pivotCfg.slot0.kD = IntakeConstants.INTAKE_PIVOT_KD;
        pivotCfg.slot0.kF = 0.0;
        // Output limits
        pivotCfg.peakOutputForward =  1.0;
        pivotCfg.peakOutputReverse = -1.0;
        // Software limits (in sensor ticks)
        pivotCfg.forwardSoftLimitEnable    = true;
        pivotCfg.forwardSoftLimitThreshold = (int) degreesToTicks(IntakeConstants.INTAKE_PIVOT_MAX_HARD_DEG);
        pivotCfg.reverseSoftLimitEnable    = true;
        pivotCfg.reverseSoftLimitThreshold = (int) degreesToTicks(IntakeConstants.INTAKE_PIVOT_MIN_HARD_DEG);
        
        pivot.configAllSettings(pivotCfg);
        pivot.setNeutralMode(NeutralMode.Brake);
        pivot.setInverted(false);
        // Assume starting at stowed position (0 degrees)
        pivot.setSelectedSensorPosition(degreesToTicks(IntakeConstants.INTAKE_PIVOT_STOWED_DEG));
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
        double clamped = Math.max(IntakeConstants.INTAKE_PIVOT_MIN_HARD_DEG,
                Math.min(IntakeConstants.INTAKE_PIVOT_MAX_HARD_DEG, deg));
        targetDeg = clamped;
        pivot.set(ControlMode.Position, degreesToTicks(clamped));
    }

    public double getPivotAngleDeg() {
        return ticksToDegrees(pivot.getSelectedSensorPosition());
    }

    public boolean pivotAtSetpoint() {
        return Math.abs(getPivotAngleDeg() - targetDeg) <= IntakeConstants.INTAKE_PIVOT_TOLERANCE_DEG;
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
        SmartDashboard.putNumber("Intake/PivotAngleDeg", getPivotAngleDeg());
        SmartDashboard.putNumber("Intake/TargetAngleDeg", targetDeg);
        SmartDashboard.putBoolean("Intake/PivotAtSetpoint", pivotAtSetpoint());
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    private static double degreesToTicks(double deg) {
        return deg * TICKS_PER_DEGREE;
    }

    private static double ticksToDegrees(double ticks) {
        return ticks / TICKS_PER_DEGREE;
    }
}
