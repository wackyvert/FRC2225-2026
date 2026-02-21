package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ClimberConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;

public class ClimberSubsystem extends SubsystemBase {

    private final SmartMotorController motor;
    private final Elevator elevator;

    private static final double SPOOL_CIRCUMFERENCE_METERS =
            Math.PI * Units.inchesToMeters(ClimberConstants.CLIMBER_SPOOL_DIAMETER_INCHES);

    public ClimberSubsystem() {
        SparkFlex spark = new SparkFlex(ClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);

        SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withIdleMode(MotorMode.BRAKE)
                .withStatorCurrentLimit(Amps.of(ClimberConstants.CLIMBER_CURRENT_LIMIT_AMPS))
                .withGearing(new MechanismGearing(ClimberConstants.CLIMBER_GEARING))
                .withMechanismCircumference(Meters.of(SPOOL_CIRCUMFERENCE_METERS))
                .withSoftLimit(
                        Meters.of(ClimberConstants.CLIMBER_MIN_HEIGHT_METERS),
                        Meters.of(ClimberConstants.CLIMBER_MAX_HEIGHT_METERS))
                .withClosedLoopController(
                        ClimberConstants.CLIMBER_KP,
                        ClimberConstants.CLIMBER_KI,
                        ClimberConstants.CLIMBER_KD)
                .withMotorInverted(false)
                .withTelemetry("ClimberMotor", TelemetryVerbosity.LOW);

        motor = new SparkWrapper(spark, DCMotor.getNeoVortex(1), motorConfig);

        ElevatorConfig elevatorConfig = new ElevatorConfig(motor)
                .withStartingHeight(Meters.of(ClimberConstants.CLIMBER_MIN_HEIGHT_METERS))
                .withHardLimits(
                        Meters.of(ClimberConstants.CLIMBER_MIN_HEIGHT_METERS),
                        Meters.of(ClimberConstants.CLIMBER_MAX_HEIGHT_METERS))
                .withMass(Kilograms.of(ClimberConstants.CLIMBER_CARRIAGE_MASS_KG))
                .withTelemetry("Climber", TelemetryVerbosity.HIGH);

        elevator = new Elevator(elevatorConfig);
    }

    /** Extend to full height (prep to climb). */
    public Command extendCommand() {
        return elevator.setHeight(Meters.of(ClimberConstants.CLIMBER_MAX_HEIGHT_METERS));
    }

    /** Retract to zero (pull robot up). */
    public Command retractCommand() {
        return elevator.setHeight(Meters.of(ClimberConstants.CLIMBER_MIN_HEIGHT_METERS));
    }

    /** Set an arbitrary target height. */
    public Command setHeightCommand(Distance height) {
        return elevator.setHeight(height);
    }

    public void runOpenLoop(double dutyCycle) {
        motor.setDutyCycle(dutyCycle);
    }

    /** Duty-cycle override for manual control. */
    public Command driveCommand(double dutyCycle) {
        return elevator.set(dutyCycle);
    }

    public Distance getHeight() {
        return elevator.getHeight();
    }

    @Override
    public void periodic() {
        elevator.updateTelemetry();
        SmartDashboard.putNumber("Climber/HeightMeters", getHeight().in(edu.wpi.first.units.Units.Meters));
    }

    @Override
    public void simulationPeriodic() {
        elevator.simIterate();
    }
}
