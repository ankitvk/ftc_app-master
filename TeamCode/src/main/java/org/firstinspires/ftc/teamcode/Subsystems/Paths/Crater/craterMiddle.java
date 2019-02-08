package org.firstinspires.ftc.teamcode.Subsystems.Paths.Crater;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;

public class craterMiddle implements Constants {

    private SpeedControlledMotor frontLeft,backLeft,frontRight,backRight;
    private BNO055_IMU imu;
    private AutonomousOpMode auto;
    private Telemetry telemetry;
    private Hardware hardware;

    private final double ROTATE_TO_GOLD_ANGLE = -85;
    private final double ROTATE_TO_GOLD_KP = 0.015/*.02725*/;
    private final double ROTATE_TO_GOLD_KI = 1.5;
    private final double ROTATE_TO_GOLD_KD = 0;

    private final double DRIVE_TO_DEPOT_DISTANCE = 64;
    private final double DRIVE_TO_DEPOT_KP = .00022;
    private final double DRIVE_TO_DEPOT_KI = 0.1;
    private final double DRIVE_TO_DEPOT_KD = 0;

    private final double ROTATE_TO_CRATER_ANGLE = -46-90;
    private final double ROTATE_TO_CRATER_KP = .015;
    private final double ROTATE_TO_CRATER_KI = 2;
    private final double ROTATE_TO_CRATER_KD = 0;

    private final double DRIVE_TO_CRATER_DISTANCE = -68;
    private final double DRIVE_TO_CRATER_KP = .000325;
    private final double DRIVE_TO_CRATER_KI = 0/*0.01*/;
    private final double DRIVE_TO_CRATER_KD = 0;

    public craterMiddle(Hardware hardware){
        this.hardware = hardware;
        frontLeft = hardware.frontLeft;
        backLeft = hardware.backLeft;
        frontRight = hardware.frontRight;
        backRight = hardware.backRight;
        imu = hardware.imu;
        auto = hardware.auto;
        telemetry = hardware.telemetry;
    }

    public void run(){
        rotateToGold();
        driveToDepot();
        //hardware.marker.setPosition(.15);
        /*rotateToCrater();
        driveToCrater();*/
    }

    private void rotateToGold(){
        double degrees = ROTATE_TO_GOLD_ANGLE;
        PIDController controlRotate = new PIDController(ROTATE_TO_GOLD_KP,ROTATE_TO_GOLD_KI,ROTATE_TO_GOLD_KD,1);
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 250)){

            double position = hardware.imu.getRelativeYaw();
            double power = controlRotate.power(degrees,position);

            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", hardware.imu.getRelativeYaw());
            telemetry.addLine(" ");
            telemetry.addData("error: ",controlRotate.getError());
            telemetry.addData("KP*error: ",controlRotate.returnVal()[0]);
            telemetry.addData("KI*i: ",controlRotate.returnVal()[1]);
            telemetry.update();

            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);

            if (Math.abs(position-degrees) <= IMU_TOLERANCE) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
            if(System.nanoTime()/1000000-beginTime/1000000>5000){
                break;
            }
        }
    }

    private void driveToDepot(){
        double distance = DRIVE_TO_DEPOT_DISTANCE;
        eReset();
        PIDController control = new PIDController(DRIVE_TO_DEPOT_KP,DRIVE_TO_DEPOT_KI,DRIVE_TO_DEPOT_KD,1);
        double ticks = (distance/(WHEEL_DIAMETER*Math.PI))*DT_GEARBOX_TICKS_PER_ROTATION;
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125)){
            double avg = hardware.frontLeft.getCurrentPosition();
            double power = control.power(ticks,avg);
            telemetry.addData("Power: ", power);
            telemetry.addData("Distance: ",ticksToDistance(avg));
            telemetry.addData("Angle: ", hardware.imu.getYaw());
            telemetry.addData("error: ",control.getError());
            telemetry.addData("KP*error: ",control.returnVal()[0]);
            telemetry.addData("KI*i: ",control.returnVal()[1]);
            telemetry.addData("KD*d: ",control.returnVal()[2]);
            telemetry.update();

            hardware.frontLeft.setPower(-power);
            hardware.backLeft.setPower(-power);
            hardware.frontRight.setPower(power);
            hardware.backRight.setPower(power);

            if (Math.abs(ticks-avg)<= distanceToTicks(DISTANCE_TOLERANCE)) {
                telemetry.addData("Distance from Target: ", Math.abs(ticks-avg));
                stopState = (System.nanoTime() - startTime) / 1000000;
            } else {
                startTime = System.nanoTime();
            }

            /*if(System.nanoTime()/1000000-beginTime/1000000>5000){
                break;
            }
*/
        }
    }

    private void rotateToCrater(){
        double degrees = ROTATE_TO_CRATER_ANGLE;
        PIDController controlRotate = new PIDController(ROTATE_TO_CRATER_KP,ROTATE_TO_CRATER_KI,ROTATE_TO_CRATER_KD,1);
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 500)){

            double position = hardware.imu.getRelativeYaw();
            double power = controlRotate.power(degrees,position);

            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", hardware.imu.getRelativeYaw());
            telemetry.addLine(" ");
            telemetry.addData("error: ",controlRotate.getError());
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
            /*if(System.nanoTime()/1000000-beginTime/1000000>3000){
                break;
            }*/
        }
    }

    private void driveToCrater(){
        double distance = DRIVE_TO_CRATER_DISTANCE;
        eReset();
        PIDController control = new PIDController(DRIVE_TO_CRATER_KP,DRIVE_TO_CRATER_KI,DRIVE_TO_CRATER_KD,1);
        double ticks = (distance/(WHEEL_DIAMETER*Math.PI))*DT_GEARBOX_TICKS_PER_ROTATION;
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 1000)){
            double avg = hardware.frontLeft.getCurrentPosition();
            double power = control.power(ticks,avg);
            telemetry.addData("Power: ", power);
            telemetry.addData("Distance: ",ticksToDistance(avg));
            telemetry.addData("Angle: ", hardware.imu.getYaw());
            telemetry.addLine(" ");
            telemetry.addData("error: ",control.getError());
            telemetry.addData("KP*error: ",control.returnVal()[0]);
            telemetry.addData("KI*i: ",control.returnVal()[1]);
            telemetry.addData("KD*d: ",control.returnVal()[2]);
            telemetry.update();

            hardware.frontLeft.setPower(-power);
            hardware.backLeft.setPower(-power);
            hardware.frontRight.setPower(power);
            hardware.backRight.setPower(power);

            if (Math.abs(ticks-avg)<= distanceToTicks(DISTANCE_TOLERANCE)) {
                telemetry.addData("Distance from Target: ", Math.abs(ticks-avg));
                stopState = (System.nanoTime() - startTime) / 1000000;
            } else {
                startTime = System.nanoTime();
            }

            /*if(System.nanoTime()/1000000-beginTime/1000000>5000){
                break;
            }
*/
        }
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

    private double distanceToTicks(double distance){
        return (distance/(WHEEL_DIAMETER*Math.PI))*DT_GEARBOX_TICKS_PER_ROTATION;
    }
    private double ticksToDistance(double ticks){
        return (ticks*(WHEEL_DIAMETER*Math.PI))/DT_GEARBOX_TICKS_PER_ROTATION;
    }
    public boolean opModeIsActive() {
        return auto.getOpModeIsActive();
    }
}
