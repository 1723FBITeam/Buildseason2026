package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

public class ShooterSubsystem extends SubsystemBase {

  // Flywheels
  private final TalonFX leftMotor = new TalonFX(26);
  private final TalonFX rightMotor = new TalonFX(27);
  private final TalonFX feederMotor = new TalonFX(28);
  private final DutyCycleOut shooterOut = new DutyCycleOut(0);
  private final DutyCycleOut feederOut = new DutyCycleOut(0);

  private final Servo hoodServo = new Servo(0); //PMW port 0
  // Servo limits (tune these!)
  private static final double HOOD_MIN = 0.15;
  private static final double HOOD_MAX = 0.85;

  public ShooterSubsystem() {
    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    rightMotor.getConfigurator().apply(motorConfigs);
  }

  // Activating shooter
  /** Spin shooter (-1.0 to 1.0) */
  public void runShooter(double speed) {
    shooterOut.Output = speed;
    feederMotor.setControl(feederOut);
    leftMotor.setControl(shooterOut);
    rightMotor.setControl(shooterOut);
  }
  


  public void stopShooter() {
    feederMotor.stopMotor();
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }


//This is the next part for later using the hood and distance logic

  // // NEW Setting hood position
  // /** 0–1 position */
  // public void setHoodPosition(double pos) {
  //   pos = MathUtil.clamp(pos, HOOD_MIN, HOOD_MAX);
  //   hoodServo.set(pos);
  // }

  // // NEW Distant logic

  // /**
  //  * Distance in METERS
  //  */
  // public void autoAim(double distanceMeters) {

  //   // ---- HOOD MAPPING ----
  //   // Example linear interpolation
  //   // 1m = low angle, 5m = high angle
  //   double hoodPos = MathUtil.interpolate(
  //       HOOD_MIN,
  //       HOOD_MAX,
  //       MathUtil.clamp((distanceMeters - 1.0) / 4.0, 0, 1)
  //   );

  //   // ---- SPEED MAPPING ----
  //   double speed = MathUtil.interpolate(
  //       0.45,  // close shot
  //       0.85,  // far shot
  //       MathUtil.clamp((distanceMeters - 1.0) / 4.0, 0, 1)
  //   );

  //   setHoodPosition(hoodPos);
  //   runShooter(speed);
  // }
}
