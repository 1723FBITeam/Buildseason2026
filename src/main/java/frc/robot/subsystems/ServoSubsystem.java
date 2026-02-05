package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
//import com.revrobotics.REVServo;

public class ServoSubsystem extends SubsystemBase {


    private final Servo testServo = new Servo(1); // PWM port 0

    public void setPosition(double position) {
        testServo.set(position);
    }

      public void levelOne() {
        testServo.set(0.0);
            System.out.println("Servo ONE called");

    }

    public void LevelTwo() {
        testServo.set(0.5);
            System.out.println("Servo TWO called");

    }

    public void LevelThree() {
        testServo.set(1.0);
            System.out.println("Servo THREE called");

    }
    

}

