package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevatorMotor = new TalonFX(ClimberConstants.Elevator_MOTOR_ID);

    public void moveUp() {
        elevatorMotor.set(0.3);
    }

    public void moveDown() {
        elevatorMotor.set(-0.3);
    }

    public void stop() {
        elevatorMotor.stopMotor();
    }
}
