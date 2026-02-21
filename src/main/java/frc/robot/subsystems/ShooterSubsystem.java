package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

/**
 * Unified shooter subsystem - controls flywheels, feeder, and indexer as a coordinated unit.
 * Turret aiming is handled separately in TurretSubsystem for independent tracking.
 */
public class ShooterSubsystem extends SubsystemBase {

  // Shooter flywheels
  private final TalonFX leftShooterMotor = new TalonFX(Constants.ShootingConstants.SHOOTER_LEFT_MOTOR);
  private final TalonFX rightShooterMotor = new TalonFX(Constants.ShootingConstants.SHOOTER_RIGHT_MOTOR);
  
  // Feeder - feeds notes into shooter
  private final TalonFX feederMotor = new TalonFX(Constants.ShootingConstants.FEEDER_MOTOR);
  
  // Indexer/Spinner - positions notes for feeding
  private final TalonFX indexerMotor = new TalonFX(Constants.ShootingConstants.SPINNER_MOTOR);
  
  private final DutyCycleOut shooterControl = new DutyCycleOut(0);

  private final Servo hoodServo = new Servo(Constants.ShootingConstants.HOOD_SERVO);
  private static final double HOOD_MIN = Constants.ShootingConstants.HOOD_MIN;
  private static final double HOOD_MAX = Constants.ShootingConstants.HOOD_MAX;

  public ShooterSubsystem() {
    // Configure shooter motors - inversions swapped to fix direction
    MotorOutputConfigs rightConfigs = new MotorOutputConfigs();
    rightConfigs.Inverted = InvertedValue.CounterClockwise_Positive;  // Flipped
    rightConfigs.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast;
    rightShooterMotor.getConfigurator().apply(rightConfigs);

    MotorOutputConfigs leftConfigs = new MotorOutputConfigs();
    leftConfigs.Inverted = InvertedValue.Clockwise_Positive;  // Flipped
    leftConfigs.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast;
    leftShooterMotor.getConfigurator().apply(leftConfigs);

    // Configure feeder motor - opposite of shooter motors
    MotorOutputConfigs feederConfigs = new MotorOutputConfigs();
    feederConfigs.Inverted = InvertedValue.Clockwise_Positive;  // Separate config for feeder
    feederConfigs.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast;
    feederMotor.getConfigurator().apply(feederConfigs);

    // Configure indexer
    MotorOutputConfigs indexerConfigs = new MotorOutputConfigs();
    indexerConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    indexerConfigs.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast;
    indexerMotor.getConfigurator().apply(indexerConfigs);
  }

  // ========== SHOOTER FLYWHEELS ==========
  
  /** Spin up shooter flywheels only (-1.0 to 1.0) */
  public void runFlywheels(double speed) {
    shooterControl.Output = speed;
    leftShooterMotor.setControl(shooterControl);
    rightShooterMotor.setControl(shooterControl);
  }

  public void stopFlywheels() {
    leftShooterMotor.stopMotor();
    rightShooterMotor.stopMotor();
  }

  // ========== FEEDER ==========
  
  /** Run feeder to push note into shooter (-1.0 to 1.0) */
  public void runFeeder(double speed) {
    feederMotor.set(speed);
  }

  public void stopFeeder() {
    feederMotor.stopMotor();
  }

  // ========== INDEXER/SPINNER ==========
  
  /** Run indexer to position notes (-1.0 to 1.0) */
  public void runIndexer(double speed) {
    indexerMotor.set(speed);
  }

  public void stopIndexer() {
    indexerMotor.stopMotor();
  }

  // ========== COMBINED OPERATIONS ==========
  
  /** Run entire shooting sequence - flywheels, feeder, and indexer */
  public void runFullShooter(double speed) {
    runFlywheels(speed);
    runFeeder(speed);
    runIndexer(speed);
  }

  /** Stop all shooter components */
  public void stopAll() {
    stopFlywheels();
    stopFeeder();
    stopIndexer();
  }

  /** Check if flywheels are at target speed (implement with velocity checking) */
  public boolean isReadyToShoot() {
    // TODO: Implement velocity checking when you tune shooter speeds
    // For now, return true
    return true;
  }

  // ========== HOOD (for future distance-based shooting) ==========

  // /** Set hood position 0-1 */
  // public void setHoodPosition(double pos) {
  //   pos = MathUtil.clamp(pos, HOOD_MIN, HOOD_MAX);
  //   hoodServo.set(pos);
  // }

  // /** Auto-aim based on distance in meters */
  // public void autoAim(double distanceMeters) {
  //   double hoodPos = MathUtil.interpolate(
  //       HOOD_MIN,
  //       HOOD_MAX,
  //       MathUtil.clamp((distanceMeters - 1.0) / 4.0, 0, 1)
  //   );
  //   
  //   double speed = MathUtil.interpolate(
  //       0.45,  // close shot
  //       0.85,  // far shot
  //       MathUtil.clamp((distanceMeters - 1.0) / 4.0, 0, 1)
  //   );
  //   
  //   setHoodPosition(hoodPos);
  //   runFlywheels(speed);
  // }

  @Override
  public void periodic() {
    // Add telemetry for debugging
    SmartDashboard.putNumber("Shooter Left Velocity", leftShooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Right Velocity", rightShooterMotor.getVelocity().getValueAsDouble());
  }
}
