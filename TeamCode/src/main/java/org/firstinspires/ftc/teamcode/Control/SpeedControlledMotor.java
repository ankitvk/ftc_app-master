package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SpeedControlledMotor implements Constants {
    private DcMotor motor;
    private int previousPos = 0;
    private long previousTime = 0;
    private double rpm = 0;
    private double power;

    PIDController PIDController;

    public SpeedControlledMotor(double KP, double KI, double KD, double maxI) {
        this.PIDController = new PIDController(KP, KI, KD, maxI);
    }


    public void init(HardwareMap hardwareMap, String name) {
        motor = hardwareMap.dcMotor.get(name);
    }

    public void setPower(double power) {
        this.power = power;
        motor.setPower(power);
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public double getRPM() {
        int deltaPos = motor.getCurrentPosition() - previousPos;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECONDS_PER_MINUTE;
        if (deltaTime*6e4 > 10) {
            rpm = (deltaPos/ DT_NEVEREST_GEARBOX)/(deltaTime);
            previousPos = motor.getCurrentPosition();
            previousTime = System.nanoTime();
        }
        return rpm;
    }

    public void setSpeed(double speed) {
        double rpm = NEVEREST_MAX_RPM *DT_NEVEREST_GEARBOX*speed;
        power = PIDController.power(rpm, getRPM());
        motor.setPower((power > 0 && getRPM() < 0) || (power < 0 && getRPM() > 0) ? 0: (power));
    }

    public void setRPM(double rpm) {
        power = PIDController.power(rpm, getRPM());
        motor.setPower((power > 0 && getRPM() < 0) || (power < 0 && getRPM() > 0) ? 0: power);

    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    public double getAngle (double ticksPerInch, double ticksPerRotation) {
        return (360 * (motor.getCurrentPosition()) % ticksPerInch) / ticksPerRotation;
    }


    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getPower() {
        return power;
    }
}
