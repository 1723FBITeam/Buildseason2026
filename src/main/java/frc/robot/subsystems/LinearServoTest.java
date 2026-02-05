package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;   // change if using Victor/Talon
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class LinearServoTest extends SubsystemBase {

    // ----- Hardware -----
    private final Spark motor;
    private final AnalogInput potentiometer;

    // ----- PID -----
    private final PIDController pid;

    // ----- Calibration Values (CHANGE THESE AFTER TESTING) -----
    // Measure these with SmartDashboard or print statements
    private double minVoltage = 0.5;  // fully retracted
    private double maxVoltage = 4.5;  // fully extended
    private boolean holdMode = true;
    // 50mm stroke actuator
    private static final double STROKE_MM = 50.0;

    // Target position in mm
    private double targetMM = 0.0;

    public LinearServoTest(int motorChannel, int analogChannel) {
        motor = new Spark(motorChannel);
        potentiometer = new AnalogInput(analogChannel);

        pid = new PIDController(0.06, 0.0, 0.0);
        pid.setTolerance(1.0); // 1mm tolerance
    }

    // ---------------- POSITION ----------------

    public double getVoltage() {
        return potentiometer.getVoltage();
    }

    public double getPositionMM() {
        double v = getVoltage();
        double percent = (v - minVoltage) / (maxVoltage - minVoltage);
        percent = MathUtil.clamp(percent, 0.0, 1.0);
        return percent * STROKE_MM;
    }

    // ---------------- CONTROL ----------------

    public void setTargetMM(double mm) {
        targetMM = MathUtil.clamp(mm, 0.0, STROKE_MM);
    }

    public void stop() {
        motor.stopMotor();
    }

    public void manualSpeed(double speed) {
    holdMode = false;
    motor.set(MathUtil.clamp(speed, -0.5, 0.5));
}
    public void holdCurrentPosition() {
    targetMM = getPositionMM();
    holdMode = true;
}


    // ---------------- PERIODIC LOOP ----------------

    @Override
public void periodic() {

    if (holdMode) {
        double output = pid.calculate(getPositionMM(), targetMM);
        output = MathUtil.clamp(output, -0.4, 0.4);

        if (pid.atSetpoint()) {
            motor.stopMotor();
        } else {
            motor.set(output);
        }
    }
}


    // ---------------- CALIBRATION HELPERS ----------------

    public void setCalibration(double minV, double maxV) {
        this.minVoltage = minV;
        this.maxVoltage = maxV;
    }

    public boolean atTarget() {
        return pid.atSetpoint();
    }
}


