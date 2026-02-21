package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.ShooterConfigFactory;
import frc.robot.constants.Constants.ShooterConstants;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerCommandRegistry;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class HoodSubsystem extends SubsystemBase {

    private final SmartMotorController motor;
    private final Pivot pivot;
    private double targetAngle = ShooterConstants.HOOD_HOME_ANGLE_DEG;

    public HoodSubsystem() {
        SparkMax spark = new SparkMax(ShooterConstants.HOOD_ID, MotorType.kBrushless);

        SmartMotorControllerConfig motorConfig = ShooterConfigFactory.createHoodMotorConfig(this);
        motor = new SparkWrapper(spark, DCMotor.getNeo550(1), motorConfig);

        PivotConfig mechConfig = ShooterConfigFactory.createHoodMechanismConfig(motor);
        pivot = new Pivot(mechConfig);

        SmartMotorControllerCommandRegistry.addCommand("Hood Home", this,
                () -> setAngleDeg(ShooterConstants.HOOD_HOME_ANGLE_DEG));
    }

    public void setAngleDeg(double degrees) {
        double clamped = Math.max(ShooterConstants.HOOD_MIN_ANGLE_DEG,
                Math.min(ShooterConstants.HOOD_MAX_ANGLE_DEG, degrees));
        targetAngle = clamped;
        pivot.setAngle(Units.Degrees.of(clamped));
    }

    public double getAngleDeg() {
        return pivot.getAngle().in(Units.Degrees);
    }

    public boolean atSetpoint() {
        return pivot.isNear(Units.Degrees.of(targetAngle),
                Units.Degrees.of(ShooterConstants.HOOD_TOLERANCE_DEG)).getAsBoolean();
    }

    @Override
    public void periodic() {
        pivot.updateTelemetry();
        SmartDashboard.putNumber("Hood/AngleDeg", getAngleDeg());
        SmartDashboard.putNumber("Hood/TargetAngleDeg", targetAngle);
        SmartDashboard.putBoolean("Hood/AtSetpoint", atSetpoint());
    }

    @Override
    public void simulationPeriodic() {
        pivot.simIterate();
    }
}
