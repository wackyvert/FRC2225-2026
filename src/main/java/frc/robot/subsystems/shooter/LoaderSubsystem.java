package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.ShooterConfigFactory;
import frc.robot.constants.Constants.ShooterConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class LoaderSubsystem extends SubsystemBase {

    private final SmartMotorController motor;
   

    public LoaderSubsystem() {
        SparkFlex spark = new SparkFlex(ShooterConstants.LOADER_ID, MotorType.kBrushless);

        SmartMotorControllerConfig config = ShooterConfigFactory.createLoaderConfig(this);
        motor = new SparkWrapper(spark, DCMotor.getNEO(1), config);

        
    }

    public void feed() {
        motor.setDutyCycle(ShooterConstants.LOADER_FEED_SPEED);
    }

    public void reverse() {
        motor.setDutyCycle(-ShooterConstants.LOADER_FEED_SPEED);
    }

    public void stop() {
        motor.setDutyCycle(0);
    }


    @Override
    public void periodic() {
        motor.updateTelemetry();
        //SmartDashboard.putBoolean("Loader/HasFuel", hasFuel());
    }

    @Override
    public void simulationPeriodic() {
        motor.simIterate();
    }
}
