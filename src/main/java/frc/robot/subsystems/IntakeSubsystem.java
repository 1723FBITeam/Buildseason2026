package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.StaticBrake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

private final TalonFX intakeLeftMotor =
    new TalonFX(Constants.INTAKE_LEFT_MOTOR);

private final TalonFX intakeRightMotor =
    new TalonFX(Constants.INTAKE_RIGHT_MOTOR);

private final TalonFX intakeActivator =
    new TalonFX(Constants.INTAKE_ACTIVATOR_MOTOR);

private SlewRateLimiter rateLimiter = new SlewRateLimiter(3);

// PID Constants
private static final double kP_UP = 0.025; // Proportional gain
private static final double kP_DOWN = 0.025; // Proportional gain
private static final double kI = 0.0; // Integral gain
private static final double kD = 0.0; // Derivative gain

private PIDController pid;

private Map<Integer, Double> targetPositions = Map.of(
            0, 1.23,
            1, 1.23);

     private int currentPositionKey = 0;

public IntakeSubsystem() {

MotorOutputConfigs leftConfigs = new MotorOutputConfigs();
leftConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
intakeLeftMotor.getConfigurator().apply(leftConfigs);

MotorOutputConfigs rightConfigs = new MotorOutputConfigs();
rightConfigs.Inverted = InvertedValue.Clockwise_Positive;
intakeRightMotor.getConfigurator().apply(rightConfigs);

 pid = new PIDController(kP_UP, kI, kD);
    intakeActivator.setControl(new StaticBrake());
    intakeActivator.setPosition(0);
}
public void runIntake(double speed) {
    intakeLeftMotor.set(speed);
    intakeRightMotor.set(speed);
}

public void stopIntake() {
    intakeLeftMotor.stopMotor();
    intakeRightMotor.stopMotor();
}

public void setIntakeUp() {
    pid.setP(kP_UP);
        currentPositionKey = targetPositions.keySet().stream().max(Integer::compareTo).orElse(currentPositionKey);
}

public void setIntakeDown() {
    pid.setP(kP_DOWN);
        currentPositionKey = targetPositions.keySet().stream().min(Integer::compareTo).orElse(currentPositionKey);    
}

    @Override
    public void periodic() {
        double currentPosition = intakeActivator.getPosition().getValueAsDouble(); // Get current position from encoder
            double pidOutput = pid.calculate(currentPosition, targetPositions.get(currentPositionKey)); // Calculate PID Output
                                                                                       
            intakeActivator.set(rateLimiter.calculate(pidOutput));
        
        SmartDashboard.putNumber("Intake ", currentPosition);
}
}