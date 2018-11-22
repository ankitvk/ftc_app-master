package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;

public class Drivetrain implements Constants {

    private SpeedControlledMotor frontLeft,backLeft,frontRight,backRight;
    private BNO055_IMU imu;
    private AutonomousOpMode auto;
    private Telemetry telemetry;
    private Hardware hardware;
    private PIDController control = new PIDController(dtKP,dtKI,dtKD,dtMaxI);

    public Drivetrain(Hardware hardware){
        this.hardware = hardware;
        frontLeft = hardware.frontLeft;
        backLeft = hardware.backLeft;
        frontRight = hardware.frontRight;
        backRight = hardware.backRight;
        imu = hardware.imu;
        auto = hardware.auto;
        telemetry = hardware.telemetry;
    }

    public void stop(){
        for(SpeedControlledMotor motor: hardware.drivetrainMotors) {
            motor.setPower(0);
        }
    }

    private void eReset() {

        for(SpeedControlledMotor motor: hardware.drivetrainMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void driveForward(double speed){
        hardware.frontLeft.setPower(-speed);
        hardware.backLeft.setPower(-speed);
        hardware.frontRight.setPower(speed);
        hardware.backRight.setPower(speed);
    }

    public void driveForwardDistance(double distance){
        eReset();
        double ticks = (distance/(WHEEL_DIAMETER*Math.PI))*0.5*NEVEREST40_COUNTS_PER_REV;
        long startTime = System.nanoTime();
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 1000)){
            double avg = ((hardware.frontLeft.getCurrentPosition())+(hardware.backLeft.getCurrentPosition()))/2;
            double power = control.power(ticks,avg);
            telemetry.addData("Power: ", power);
            telemetry.addData("Distance: ",ticksToDistance(avg));
            telemetry.addData("Angle: ", hardware.imu.getYaw());

            hardware.frontLeft.setPower(power);
            hardware.backLeft.setPower(power);
            hardware.frontRight.setPower(-power);
            hardware.backRight.setPower(-power);

            if (Math.abs(ticks-avg)<= distanceToTicks(DISTANCE_TOLERANCE)) {
                telemetry.addData("Distance from Target: ", Math.abs(ticks-avg));
                stopState = (System.nanoTime() - startTime) / 1000000;
            } else {
                startTime = System.nanoTime();
            }
            telemetry.update();
        }
    }

    public void driveForTime(double power, double time){
        eReset();
        long startTime = System.nanoTime();
        long stopState = 0;
        while(stopState <= time){
            hardware.frontLeft.setPower(-power);
            hardware.backLeft.setPower(-power);
            hardware.frontRight.setPower(power);
            hardware.backRight.setPower(power);

            stopState = (System.nanoTime() - startTime) / 1000000;
        }
        stop();
    }

    public void rotateForTime(double power, double time){
        eReset();
        long startTime = System.nanoTime();
        long stopState = 0;
        while(stopState <= time){
            hardware.frontLeft.setPower(-power);
            hardware.backLeft.setPower(-power);
            hardware.frontRight.setPower(-power);
            hardware.backRight.setPower(-power);

            stopState = (System.nanoTime() - startTime) / 1000000;
        }
        stop();
    }

    public void rotateToAbsoluteAngle(double desire){
        double degrees = desire;
        PIDController controlRotate = new PIDController(.003,0.00001,0,1);
        long startTime = System.nanoTime();
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 1000)){
            double position = hardware.imu.getRelativeYaw();
            double power = controlRotate.power(degrees,position);
            if(Math.abs(power)<.3){
                if(Math.abs(power)<.01){
                    break;
                }
                else if(power>=0){
                    power = .3;
                }
                else {
                    power = -.3;
                }
            }
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", hardware.imu.getRelativeYaw());
            telemetry.addLine(" ");
            telemetry.addData("KP*error: ",controlRotate.returnVal()[0]);
            telemetry.addData("KI*i: ",controlRotate.returnVal()[1]);
            telemetry.addData("KD*d: ",controlRotate.returnVal()[2]);
            telemetry.update();
            hardware.frontLeft.setPower(power);
            hardware.backLeft.setPower(power);
            hardware.frontRight.setPower(power);
            hardware.backRight.setPower(power);

            if (Math.abs(position-degrees) <= IMU_TOLERANCE) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
        }
        stop();
    }

    public void rotateToBigAbsoluteAngle(double desire){
        double degrees = desire;
        PIDController controlRotate = new PIDController(.0025,0.00001,0,1);
        long startTime = System.nanoTime();
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 1000)){
            double position = hardware.imu.getRelativeYaw();
            double power = controlRotate.power(degrees,position);
            if(Math.abs(power)<.35){
                if(Math.abs(power)<.01){
                    break;
                }
                else if(power>=0){
                    power = .35;
                }
                else {
                    power = -.35;
                }
            }
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", hardware.imu.getRelativeYaw());
            telemetry.addLine(" ");
            telemetry.addData("KP*error: ",controlRotate.returnVal()[0]);
            telemetry.addData("KI*i: ",controlRotate.returnVal()[1]);
            telemetry.addData("KD*d: ",controlRotate.returnVal()[2]);
            telemetry.update();
            hardware.frontLeft.setPower(power);
            hardware.backLeft.setPower(power);
            hardware.frontRight.setPower(power);
            hardware.backRight.setPower(power);

            if (Math.abs(Math.abs(position)-Math.abs(degrees)) <= IMU_TOLERANCE) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
        }
        stop();
    }

    public void rotate(double speed){
        hardware.frontLeft.setPower(speed);
        hardware.backLeft.setPower(speed);
        hardware.frontRight.setPower(speed);
        hardware.backRight.setPower(speed);
    }

    public boolean opModeIsActive() {
        return auto.getOpModeIsActive();
    }

    public double distanceToTicks(double distance){
        return (distance/(WHEEL_DIAMETER*Math.PI))*0.5*NEVEREST40_COUNTS_PER_REV;
    }
    public double ticksToDistance(double ticks){
        return (ticks*(WHEEL_DIAMETER*Math.PI)*2)/NEVEREST40_COUNTS_PER_REV;
    }

    public void rotateToRelativeAngle(double degrees){
        rotateToAbsoluteAngle(hardware.imu.getYaw()+degrees);
    }

}
