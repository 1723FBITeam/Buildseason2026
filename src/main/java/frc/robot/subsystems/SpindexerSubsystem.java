package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpindexerSubsystem extends SubsystemBase {

private final TalonFX indexMotor = new TalonFX(Constants.INDEX_MOTOR);

public SpindexerSubsystem() {
    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    indexMotor.getConfigurator().apply(motorConfigs);
  }

public void rotate(double speed) {
        indexMotor.set(speed);
    }

    public void stop() {
        indexMotor.stopMotor();
    }


}
