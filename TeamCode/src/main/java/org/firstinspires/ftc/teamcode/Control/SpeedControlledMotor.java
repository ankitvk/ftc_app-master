package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelUtils.MaelUtils;

public class SpeedControlledMotor implements Constants {
    private DcMotor motor;
    private int previousPos = 0;
    private long previousTime = 0;
    private double rpm = 0;
    private double currVelocity = 0;


    public PIDController PIDController;

    public SpeedControlledMotor(double KP, double KI, double KD, double maxI) {
        this.PIDController = new PIDController(KP, KI, KD, maxI);
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
            rpm = (deltaPos/ NEVEREST20_COUNTS_PER_REV)/(deltaTime);
            previousPos = motor.getCurrentPosition();
            previousTime = System.nanoTime();
        }
        return rpm;
    }

    public double getVelocity(){
        int deltaPos = getCurrentPosition() - previousPos;
        //double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_SEC;
        double deltaTime = MaelUtils.getDeltaTime();
        if (deltaTime*6e4 > 10) {
            currVelocity = (deltaPos/ NEVEREST20_COUNTS_PER_REV)/(deltaTime);
            previousPos = motor.getCurrentPosition();
            previousTime = System.nanoTime();
        }
        return currVelocity;
    }

    public void setVelocity(double velocity){
        double targetVelocity = getTargetVelocity(velocity);
        double power = PIDController.power(targetVelocity, getVelocity());
        setPower(power);
    }

    public double getTargetVelocity(double velocity){
        double target = NEVEREST20_COUNTS_PER_REV * velocity;
        return target;
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
        motor.setPower(power);
    }

    public void setRPM(double rpm) {
        double power = PIDController.power(rpm, getRPM());
        //motor.setPower((power > 0 && getRPM() > 0) || (power < 0 && getRPM() < 0)  ? 0: power);
        motor.setPower(power);

    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    public double getAngle ( double ticksPerRotation) {
        return 360 * motor.getCurrentPosition()/ ticksPerRotation;
    }

    public void setReverseMode(){
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setForwardMode(){
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getPower() {
        return motor.getPower();
    }


}
