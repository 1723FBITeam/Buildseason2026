package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    private final SparkMax turretMotor =
        new SparkMax(Constants.TURRET_MOTOR, MotorType.kBrushed);

    public void rotate(double speed) {
        turretMotor.set(speed);
    }

    public void stop() {
        turretMotor.stopMotor();
    }
}


        //SparkMaxConfig config = new SparkMaxConfig();
        
        //config.idleMode(IdleMode.kBrake);
        //config.smartCurrentLimit(20);

        //turretMotor.configure(config);



        //turretMotor.restoreFactoryDefaults();
        //turretMotor.setIdleMode (SparkMax.IdleMode.kBrake);
        //turretMotor.setSmartCurrentLimit(20);
    

