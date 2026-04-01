package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.ShooterConfigFactory;
import frc.robot.constants.Constants.Global;
import frc.robot.constants.Constants.ShooterConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class LoaderSubsystem extends SubsystemBase {
    private enum LoaderRequest {
        STOP,
        FEED,
        MANUAL_REVERSE
    }

    private final SmartMotorController motor;
    private LoaderRequest requestedAction = LoaderRequest.STOP;
    private double appliedDutyCycle = 0.0;
    private boolean autoReversing = false;
    private int jamLoops = 0;
    private int reverseLoopsRemaining = 0;
    private int recoveryLoopsRemaining = 0;
    private boolean jamDetected = false;

    public LoaderSubsystem() {
        SparkFlex spark = new SparkFlex(ShooterConstants.LOADER_ID, MotorType.kBrushless);

        SmartMotorControllerConfig config = ShooterConfigFactory.createLoaderConfig(this);
        motor = new SparkWrapper(spark, DCMotor.getNEO(1), config);
    }

    public void feed() {
        requestedAction = LoaderRequest.FEED;
        if (!autoReversing) {
            applyDutyCycle(ShooterConstants.LOADER_FEED_SPEED);
        }
    }

    public void reverse() {
        requestedAction = LoaderRequest.MANUAL_REVERSE;
        clearJamState();
        applyDutyCycle(-ShooterConstants.LOADER_REVERSE_SPEED);
    }

    public void stop() {
        requestedAction = LoaderRequest.STOP;
        clearJamState();
        applyDutyCycle(0.0);
    }

    @Override
    public void periodic() {
        motor.updateTelemetry();

        double statorCurrentAmps = motor.getStatorCurrent().in(Amps);
        double velocityRps = motor.getMechanismVelocity().in(RotationsPerSecond);

        switch (requestedAction) {
            case FEED:
                runFeedControl(statorCurrentAmps, velocityRps);
                break;
            case MANUAL_REVERSE:
                applyDutyCycle(-ShooterConstants.LOADER_REVERSE_SPEED);
                break;
            case STOP:
            default:
                applyDutyCycle(0.0);
                break;
        }

        SmartDashboard.putNumber("Loader/DutyCycle", appliedDutyCycle);
        SmartDashboard.putNumber("Loader/StatorCurrentAmps", statorCurrentAmps);
        SmartDashboard.putNumber("Loader/VelocityRps", velocityRps);
        SmartDashboard.putBoolean("Loader/JamDetected", jamDetected);
        SmartDashboard.putBoolean("Loader/AutoReversing", autoReversing);
        SmartDashboard.putNumber("Loader/JamDetectionLoops", jamLoops);
    }

    @Override
    public void simulationPeriodic() {
        motor.simIterate();
    }

    private void runFeedControl(double statorCurrentAmps, double velocityRps) {
        if (autoReversing) {
            applyDutyCycle(-ShooterConstants.LOADER_REVERSE_SPEED);
            reverseLoopsRemaining--;
            if (reverseLoopsRemaining <= 0) {
                autoReversing = false;
                jamDetected = false;
                jamLoops = 0;
                recoveryLoopsRemaining = secondsToLoops(ShooterConstants.LOADER_JAM_RECOVERY_TIME_S);
            }
            return;
        }

        applyDutyCycle(ShooterConstants.LOADER_FEED_SPEED);

        if (recoveryLoopsRemaining > 0) {
            recoveryLoopsRemaining--;
            jamLoops = 0;
            jamDetected = false;
            return;
        }

        boolean currentHigh = statorCurrentAmps >= ShooterConstants.LOADER_JAM_CURRENT_THRESHOLD_AMPS;
        boolean stalled = Math.abs(velocityRps) <= ShooterConstants.LOADER_STALL_VELOCITY_RPS;

        if (currentHigh && stalled) {
            jamLoops++;
            if (jamLoops >= secondsToLoops(ShooterConstants.LOADER_JAM_DETECTION_TIME_S)) {
                jamDetected = true;
                autoReversing = true;
                reverseLoopsRemaining = secondsToLoops(ShooterConstants.LOADER_AUTO_REVERSE_TIME_S);
                jamLoops = 0;
            }
        } else {
            jamLoops = 0;
            jamDetected = false;
        }
    }

    private void clearJamState() {
        autoReversing = false;
        jamDetected = false;
        jamLoops = 0;
        reverseLoopsRemaining = 0;
        recoveryLoopsRemaining = 0;
    }

    private void applyDutyCycle(double dutyCycle) {
        appliedDutyCycle = dutyCycle;
        motor.setDutyCycle(dutyCycle);
    }

    private int secondsToLoops(double seconds) {
        return Math.max(1, (int) Math.ceil(seconds / Global.LOOP_TIME_S));
    }
}
