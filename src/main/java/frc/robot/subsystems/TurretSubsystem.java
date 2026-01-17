package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class TurretSubsystem extends SubsystemBase {

    private final SparkMax turretMotor =
        new SparkMax(Constants.TURRET_MOTOR, MotorType.kBrushed);

    private final PIDController aimPID = new PIDController(0.02, 0.0, 0.001);

    private final NetworkTable limelightTable =
        NetworkTableInstance.getDefault().getTable("limelight-turrt");

    public void rotate(double speed) {
        turretMotor.set(speed);
    }

    public void stop() {
        turretMotor.stopMotor();
    }

    /** Called by the command to auto-aim */
    public void aimAtTag() {
        limelightTable.getEntry("pipeline").setDouble(0.0);

        double tx = limelightTable.getEntry("tx").getDouble(0.0);

        // if theres no target it doesnt
        if (tx == 0) {
            turretMotor.set(0);
            return;
        }

        double turretSpeed = aimPID.calculate(tx, 0.0);

        // Clamp speed
        turretSpeed = Math.max(-0.4, Math.min(0.4, turretSpeed));

        turretMotor.set(turretSpeed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Limelight TX",
            limelightTable.getEntry("tx").getDouble(0.0));
    }
}



        //SparkMaxConfig config = new SparkMaxConfig();
        
        //config.idleMode(IdleMode.kBrake);
        //config.smartCurrentLimit(20);

        //turretMotor.configure(config);



        //turretMotor.restoreFactoryDefaults();
        //turretMotor.setIdleMode (SparkMax.IdleMode.kBrake);
        //turretMotor.setSmartCurrentLimit(20);
    

