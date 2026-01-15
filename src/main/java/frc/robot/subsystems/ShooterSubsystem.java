package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ShooterSubsystem extends SubsystemBase {

     private final TalonFX leftMotor = new TalonFX(26);  // CAN ID 20
  private final TalonFX rightMotor = new TalonFX(27); // CAN ID 21

  private final DutyCycleOut shooterOut = new DutyCycleOut(0);


  public ShooterSubsystem() {
    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
motorConfigs.Inverted = InvertedValue.Clockwise_Positive;

rightMotor.getConfigurator().apply(motorConfigs);; // motors face each other
  }
  
  /** Spin shooter (-1.0 to 1.0) */
  public void runShooter(double speed) {
    shooterOut.Output = speed;
    leftMotor.setControl(shooterOut);
    rightMotor.setControl(shooterOut);
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
    }
}

