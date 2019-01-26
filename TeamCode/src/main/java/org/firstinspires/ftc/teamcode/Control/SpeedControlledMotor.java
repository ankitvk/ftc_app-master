package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SpeedControlledMotor implements Constants {
    private DcMotor motor;
    private int previousPos = 0;
    private long previousTime = 0;
    private double rpm = 0;

    private double targetPower;
    private double prevPos = 0;
    private double prevRate;
    private double rpmIntegral = 0;
    private double rpmDerivative = 0;
    private double rpmPreviousError = 0;
    private double motorPower;
    private double currentMax, currentMin;
    private double currentZero;
    private int direction = 1;
    private double minPower = 0;
    private double minPosition, maxPosition;
    private boolean
            limitDetection = false,
            positionDetection = false,
            halfDetectionMin = false,
            halfDetectionMax = false,
            closedLoop = false;


    private double kp,ki,kd,maxI;

    PIDController PIDController;

    public SpeedControlledMotor(double KP, double KI, double KD, double maxI) {
        this.PIDController = new PIDController(KP, KI, KD, maxI);
        this.kp = KP;
        this.ki = KI;
        this.kd = KD;
        this.maxI = maxI;
    }


    public void init(HardwareMap hardwareMap, String name) {
        motor = hardwareMap.dcMotor.get(name);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public double getRPM() {
        int deltaPos = motor.getCurrentPosition() - previousPos;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECONDS_PER_MINUTE;
        if (deltaTime*6e4 > 10) {
            rpm = (deltaPos/ DT_GEARBOX_TICKS_PER_ROTATION)/(deltaTime);
            previousPos = motor.getCurrentPosition();
            previousTime = System.nanoTime();
        }
        return rpm;
    }

    double rpmTemp = 0;

    public double getRpmTemp() {
        return rpmTemp;
    }

    public void setSpeed(double speed) {
        double rpm = DT_MAX_RPM*speed;
        rpmTemp = rpm;
        double power = PIDController.power(rpm, getRPM());
        //motor.setPower((power > 0 && getRPM() > 0) || (power < 0 && getRPM() < 0) ? 0: (power));
        if(speed==0){
            motor.setPower(0);
        }
        else{
            motor.setPower(power);
        }
    }

    public void setRPM(double rpm) {
        double power = PIDController.power(rpm, getRPM());
        //motor.setPower((power > 0 && getRPM() > 0) || (power < 0 && getRPM() < 0)  ? 0: power);
        motor.setPower(power);

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
        return motor.getPower();
    }

    public void setVelocity(double power) {
        targetPower = power;
        motorPower = calculateVelocityCorrection();
        if (!closedLoop) motorPower = targetPower;
        /*if (limitDetection) {
            if (minLim != null && minLim.isPressed() && power < 0 ||
                    maxLim != null && maxLim.isPressed() && power > 0)
                motorPower = 0;
            else if (minLim != null && minLim.isPressed()
                    && power < 0 && maxLim == null)
                motorPower = 0;
        }*/

        /*else if (positionDetection) {
            if ((motor.getCurrentPosition() < minPosition && power < 0) ||
                    (motor.getCurrentPosition() > maxPosition && power > 0))
                power = 0;
            else if (motor.getCurrentPosition() < minPosition && power < 0)
                motorPower = 0;
        }
        else if (halfDetectionMin) {
            if (minLim.isPressed()) {
                currentZero = motor.getCurrentPosition();
                currentMax = currentZero + maxPosition;
            }
            if (minLim != null && minLim.isPressed() && power < 0) motorPower = 0;
            else if (motor.getCurrentPosition() > currentMax && power > 0) motorPower = 0;
        }
        else if (halfDetectionMax) {
            if (maxLim.isPressed()) {
                currentZero = motor.getCurrentPosition();
                currentMin = currentZero - minPosition;
            }
            if (maxLim != null && maxLim.isPressed() && power >0) motorPower = 0;
            else if (motor.getCurrentPosition() < currentMin && power < 0) motorPower = 0;
        }*/
        if (Math.abs(motorPower) < minPower && minPower != 0) motorPower = 0;
        motor.setPower(motorPower);
    }

    private double calculateVelocityCorrection() {
        double error, setRPM, currentRPM, tChange;
        tChange = System.nanoTime() - previousTime;
        tChange /= 1e9;
        setRPM = getRPM() * targetPower;
        currentRPM = getVelocity();
        error = setRPM - currentRPM;
        rpmIntegral += error * tChange;
        rpmDerivative = (error - rpmPreviousError) / tChange;
        double power = (targetPower) + (direction * ((error * kp) +
                (rpmIntegral * ki) + (rpmDerivative * kd)));
        rpmPreviousError = error;
        previousTime = System.nanoTime();
        return power;
    }

    public double getVelocity() {
        double deltaPosition = getCurrentPosition() - prevPos;
        double tChange = System.nanoTime() - previousTime;
        previousTime = System.nanoTime();
        tChange = tChange / 1e9;
        prevPos = getCurrentPosition();
        double rate = deltaPosition / tChange;
        rate = (rate * 60) / DT_GEARBOX_TICKS_PER_ROTATION;
        if (rate != 0) return rate;
        else {
            prevRate = rate;
            return prevRate;
        }
    }

}
