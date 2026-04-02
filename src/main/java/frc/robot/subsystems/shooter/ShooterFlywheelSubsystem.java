package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.ShooterConfigFactory;
import frc.robot.constants.Constants.ShooterConstants;
import java.util.function.Supplier;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterFlywheelSubsystem extends SubsystemBase {
    private static final String TUNE_PREFIX = "Flywheel/Tuning/";

    private final SmartMotorController motor;
    private final FlyWheel flywheel;
    private final RelativeEncoder leaderEncoder;
    private double setpointRPM = 0.0;
    private double tuningKp = ShooterConstants.FLYWHEEL_KP;
    private double tuningKi = ShooterConstants.FLYWHEEL_KI;
    private double tuningKd = ShooterConstants.FLYWHEEL_KD;
    private double tuningKs = ShooterConstants.FLYWHEEL_KS;
    private double tuningKv = ShooterConstants.FLYWHEEL_KV;
    private double tuningTargetRPM = 0.0;
    private boolean tuningModeEnabled = false;
    private boolean previousTuningModeEnabled = false;
    private boolean tuningApplied = false;

    public ShooterFlywheelSubsystem() {
        SparkFlex Lleader = new SparkFlex(ShooterConstants.FLYWHEEL_ID, MotorType.kBrushless);
        SparkFlex Rfollower = new SparkFlex(ShooterConstants.FLYWHEEL_FOLLOWER_ID, MotorType.kBrushless);
        leaderEncoder = Lleader.getEncoder();

        // Hardware following — same shaft, so REV native follow is correct.
        // YAMS only manages the LLleader; the Rfollower mirrors it at the hardware level.
        SparkFlexConfig RfollowerConfig = new SparkFlexConfig();
        RfollowerConfig.follow(Lleader, ShooterConstants.FLYWHEEL_FOLLOWER_INVERTED);
        try {
            Rfollower.configure(RfollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } catch (Exception e) {
            System.out.println("[Flywheel] Rfollower configure failed (not connected?): " + e.getMessage());
        }

        SmartMotorControllerConfig motorConfig = ShooterConfigFactory.createFlywheelMotorConfig(this);
        // Pass DCMotor.getNeoVortex(2) so the sim model reflects both motors
        motor = new SparkWrapper(Lleader, DCMotor.getNeoVortex(2), motorConfig);

        FlyWheelConfig mechConfig = ShooterConfigFactory.createFlywheelMechanismConfig(motor);
        flywheel = new FlyWheel(mechConfig);

        publishTuningDefaults();
    }

    public AngularVelocity getVelocity() {
        return flywheel.getSpeed();
    }

    public double getRPM() {
        return flywheel.getSpeed().in(Units.RPM);
    }

    public Command setVelocity(AngularVelocity speed) {
        setpointRPM = speed.in(Units.RPM);
        return flywheel.setSpeed(speed);
    }

    public void setVelocitySetpoint(AngularVelocity speed) {
        setpointRPM = speed.in(Units.RPM);
        motor.startClosedLoopController();
        motor.setVelocity(speed);
    }

    public Command setVelocity(Supplier<AngularVelocity> speedSupplier) {
        return flywheel.setSpeed(() -> {
            AngularVelocity v = speedSupplier.get();
            setpointRPM = v.in(Units.RPM);
            return v;
        });
    }

    public void runOpenLoop(double dutyCycle) {
        motor.setDutyCycle(dutyCycle);
    }

    public Command setDutyCycle(double dutyCycle) {
        return flywheel.set(dutyCycle);
    }

    public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
        return flywheel.set(dutyCycleSupplier);
    }

    public boolean atSpeed() {
        return Math.abs(getRPM() - setpointRPM) <= ShooterConstants.FLYWHEEL_TOLERANCE_RPM;
    }

    @Override
    public void periodic() {
        handleDashboardTuning();
        flywheel.updateTelemetry();
        SmartDashboard.putNumber("Flywheel/RawLeaderEncoderRPM", leaderEncoder.getVelocity());
        SmartDashboard.putNumber("Flywheel/RPM", getRPM());
        SmartDashboard.putNumber("Flywheel/SetpointRPM", setpointRPM);
        SmartDashboard.putNumber("Flywheel/ErrorRPM", setpointRPM - getRPM());
        SmartDashboard.putBoolean("Flywheel/AtSpeed", atSpeed());
        SmartDashboard.putNumber("Flywheel/AppliedDutyCycle", motor.getDutyCycle());
        SmartDashboard.putBoolean("Flywheel/Tuning/Applied", tuningApplied);
        SmartDashboard.putBoolean("Flywheel/Tuning/Enabled", tuningModeEnabled);
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }

    private void publishTuningDefaults() {
        SmartDashboard.putBoolean(TUNE_PREFIX + "Enabled", false);
        SmartDashboard.putNumber(TUNE_PREFIX + "TargetRPM", 0.0);
        SmartDashboard.putNumber(TUNE_PREFIX + "kP", tuningKp);
        SmartDashboard.putNumber(TUNE_PREFIX + "kI", tuningKi);
        SmartDashboard.putNumber(TUNE_PREFIX + "kD", tuningKd);
        SmartDashboard.putNumber(TUNE_PREFIX + "kS", tuningKs);
        SmartDashboard.putNumber(TUNE_PREFIX + "kV", tuningKv);
    }

    private void handleDashboardTuning() {
        previousTuningModeEnabled = tuningModeEnabled;
        tuningModeEnabled = SmartDashboard.getBoolean(TUNE_PREFIX + "Enabled", false);

        double requestedKp = SmartDashboard.getNumber(TUNE_PREFIX + "kP", tuningKp);
        double requestedKi = SmartDashboard.getNumber(TUNE_PREFIX + "kI", tuningKi);
        double requestedKd = SmartDashboard.getNumber(TUNE_PREFIX + "kD", tuningKd);
        double requestedKs = SmartDashboard.getNumber(TUNE_PREFIX + "kS", tuningKs);
        double requestedKv = SmartDashboard.getNumber(TUNE_PREFIX + "kV", tuningKv);
        double requestedTargetRPM = SmartDashboard.getNumber(TUNE_PREFIX + "TargetRPM", tuningTargetRPM);

        if (requestedKp != tuningKp || requestedKi != tuningKi || requestedKd != tuningKd) {
            tuningKp = requestedKp;
            tuningKi = requestedKi;
            tuningKd = requestedKd;
            motor.setFeedback(tuningKp, tuningKi, tuningKd);
            tuningApplied = true;
        }

        if (requestedKs != tuningKs || requestedKv != tuningKv) {
            tuningKs = requestedKs;
            tuningKv = requestedKv;
            motor.setFeedforward(tuningKs, tuningKv, 0.0, 0.0);
            tuningApplied = true;
        }

        tuningTargetRPM = requestedTargetRPM;

        if (tuningModeEnabled) {
            setpointRPM = tuningTargetRPM;
            motor.startClosedLoopController();
            motor.setVelocity(Units.RPM.of(tuningTargetRPM));
        } else if (previousTuningModeEnabled) {
            setpointRPM = 0.0;
            motor.stopClosedLoopController();
            motor.setDutyCycle(0.0);
        }
    }
}
