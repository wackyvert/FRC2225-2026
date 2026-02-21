package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

    private final SmartMotorController motor;
    private final FlyWheel flywheel;
    private double setpointRPM = 0.0;

    public ShooterFlywheelSubsystem() {
        SparkFlex leader = new SparkFlex(ShooterConstants.FLYWHEEL_ID, MotorType.kBrushless);
        SparkFlex follower = new SparkFlex(ShooterConstants.FLYWHEEL_FOLLOWER_ID, MotorType.kBrushless);

        // Hardware following — same shaft, so REV native follow is correct.
        // YAMS only manages the leader; the follower mirrors it at the hardware level.
        SparkFlexConfig followerConfig = new SparkFlexConfig();
        followerConfig.follow(leader, ShooterConstants.FLYWHEEL_FOLLOWER_INVERTED);
        try {
            follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } catch (Exception e) {
            System.out.println("[Flywheel] Follower configure failed (not connected?): " + e.getMessage());
        }

        SmartMotorControllerConfig motorConfig = ShooterConfigFactory.createFlywheelMotorConfig(this);
        // Pass DCMotor.getNeoVortex(2) so the sim model reflects both motors
        motor = new SparkWrapper(leader, DCMotor.getNeoVortex(2), motorConfig);

        FlyWheelConfig mechConfig = ShooterConfigFactory.createFlywheelMechanismConfig(motor);
        flywheel = new FlyWheel(mechConfig);
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

    public Command setVelocity(Supplier<AngularVelocity> speedSupplier) {
        return flywheel.setSpeed(() -> {
            AngularVelocity v = speedSupplier.get();
            setpointRPM = v.in(Units.RPM);
            return v;
        });
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
        flywheel.updateTelemetry();
        SmartDashboard.putNumber("Flywheel/RPM", getRPM());
        SmartDashboard.putBoolean("Flywheel/AtSpeed", atSpeed());
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }
}
