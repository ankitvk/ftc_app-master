package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;

public class Drivetrain implements Constants {

    private SpeedControlledMotor frontLeft,backLeft,frontRight,backRight;
    private BNO055_IMU imu;
    private AutonomousOpMode auto;
    private Hardware hardware;
    public Telemetry telemetry;
    public PIDController control = new PIDController(dtKP,dtKI,dtKD,dtMaxI);

    public Drivetrain(Hardware hardware){
        this.hardware = hardware;
        frontLeft = hardware.frontLeft;
        backLeft = hardware.backLeft;
        frontRight = hardware.frontRight;
        backRight = hardware.backRight;
        imu = hardware.imu;
        auto = hardware.auto;
    }

    public void stop(){
        for(SpeedControlledMotor motor: hardware.drivetrainMotors) {
            motor.setPower(0);
        }
    }

    public void eReset() {

        for(SpeedControlledMotor motor: hardware.drivetrainMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void driveForward(double speed){
        hardware.frontLeft.setPower(speed);
        hardware.backLeft.setPower(speed);
        hardware.frontRight.setPower(-speed);
        hardware.backRight.setPower(-speed);
    }

    public void driveForwardDistance(double speed, double distance){
        eReset();
        double ticks = (distance/(WHEEL_DIAMETER*Math.PI))*0.5*NEVEREST40_COUNTS_PER_REV;
        long startTime = System.nanoTime();
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 1000)){
            double power = control.power(ticks,hardware.frontLeft.getCurrentPosition());
            hardware.frontLeft.setPower(power);
            hardware.backLeft.setPower(power);
            hardware.frontRight.setPower(power);
            hardware.backRight.setPower(power);

            if (Math.abs(frontRight.getCurrentPosition() - -ticks) <= DISTANCE_TOLERANCE) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            } else {
                startTime = System.nanoTime();
            }
        }
    }

    public void rotateToAngle(double radians){
        double initialYaw = imu.getYaw();
        double error = Math.abs((initialYaw+radians)-initialYaw);
        double power;
        while(error>0.08){
            power = control.power(initialYaw+radians,imu.getYaw());
            hardware.frontLeft.setPower(power);
            hardware.backLeft.setPower(power);
            hardware.frontRight.setPower(power);
            hardware.backRight.setPower(power);
        }

    }

    public void rotate(double speed, boolean counterClockwise){

        if(counterClockwise){
            hardware.frontLeft.setPower(speed);
            hardware.backLeft.setPower(speed);
            hardware.frontRight.setPower(speed);
            hardware.backRight.setPower(speed);
        }
        else{
            hardware.frontLeft.setPower(-speed);
            hardware.backLeft.setPower(-speed);
            hardware.frontRight.setPower(-speed);
            hardware.backRight.setPower(-speed);
        }
    }

    public boolean opModeIsActive() {
        return auto.getOpModeIsActive();
    }
}
