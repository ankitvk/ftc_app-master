package org.firstinspires.ftc.teamcode.Subsystems.Paths.Crater;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;

public class craterLeft implements Constants {

    private SpeedControlledMotor frontLeft,backLeft,frontRight,backRight;
    private BNO055_IMU imu;
    private AutonomousOpMode auto;
    private Telemetry telemetry;
    private Hardware hardware;

    private final double ROTATE_TO_GOLD_ANGLE = 25;
    private final double ROTATE_TO_GOLD_KP = .02725;
    private final double ROTATE_TO_GOLD_KI = 1.5;
    private final double ROTATE_TO_GOLD_KD = 0;

    private final double DRIVE_TO_GOLD_DISTANCE = 35;
    private final double DRIVE_TO_GOLD_KP = .00028;
    private final double DRIVE_TO_GOLD_KI = 0.01;
    private final double DRIVE_TO_GOLD_KD = 0;

    private final double DRIVE_BACK_DISTANCE = 15;
    private final double DRIVE_BACK_KP = .00028;
    private final double DRIVE_BACK_KI = 0.01;
    private final double DRIVE_BACK_KD = 0;

    private final double ROTATE_90_ANGLE = 90;
    private final double ROTATE_90_KP = .02725;
    private final double ROTATE_90_KI = 1.5;
    private final double ROTATE_90_KD = 0;

    private final double DRIVE_TO_SAMPLE_2_DISTANCE = 60;
    private final double DRIVE_TO_SAMPLE_2_KP = .00028;
    private final double DRIVE_TO_SAMPLE_2_KI = 0.01;
    private final double DRIVE_TO_SAMPLE_2_KD = 0;

    private final double ROTATE_TO_DEPOT_ANGLE = 135;
    private final double ROTATE_TO_DEPOT_KP = .02725;
    private final double ROTATE_TO_DEPOT_KI = 1.5;
    private final double ROTATE_TO_DEPOT_KD = 0;

    private final double DRIVE_TO_DEPOT_DISTANCE = 25;
    private final double DRIVE_TO_DEPOT_KP = .00028;
    private final double DRIVE_TO_DEPOT_KI = 0.01;
    private final double DRIVE_TO_DEPOT_KD = 0;

    private final double DRIVE_BACK_TO_GOLD_DISTANCE = -30;
    private final double DRIVE_BACK_TO_GOLD_KP = .00028;
    private final double DRIVE_BACK_TO_GOLD_KI = 0.01;
    private final double DRIVE_BACK_TO_GOLD_KD = 0;

    private final double ROTATE_TO_GOLD_2_ANGLE = 45;
    private final double ROTATE_TO_GOLD_2_KP = .02725;
    private final double ROTATE_TO_GOLD_2_KI = 1.5;
    private final double ROTATE_TO_GOLD_2_KD = 0;

    private final double DRIVE_TO_GOLD_2_DISTANCE = -15;
    private final double DRIVE_TO_GOLD_2_KP = .00028;
    private final double DRIVE_TO_GOLD_2_KI = 0.01;
    private final double DRIVE_TO_GOLD_2_KD = 0;

    public craterLeft(Hardware hardware) {
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
        driveToGold();
        driveBack();
        rotate90();
        driveToSample2();
        rotateToDepot();
        driveToDepot();
        stop();
        hardware.intake.setPower(.75);
        sleep(1000);
        hardware.intake.setPower(0);
        driveBackToGold();
        rotateToGold2();
        driveToGold2();
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
            if(System.nanoTime()/1000000-beginTime/1000000>2000){
                break;
            }
        }
    }

    private void driveBack(){
        double distance = DRIVE_BACK_DISTANCE;
        eReset();
        PIDController control = new PIDController(DRIVE_BACK_KP,DRIVE_BACK_KI,DRIVE_BACK_KD,1);
        double ticks = (distance/(WHEEL_DIAMETER*Math.PI))*DT_NEVEREST_GEARBOX;
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
            telemetry.update();

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            if (Math.abs(ticks-avg)<= distanceToTicks(DISTANCE_TOLERANCE)) {
                telemetry.addData("Distance from Target: ", Math.abs(ticks-avg));
                stopState = (System.nanoTime() - startTime) / 1000000;
            } else {
                startTime = System.nanoTime();
            }

            if(System.nanoTime()/1000000-beginTime/1000000>5500){
                break;
            }

        }
    }

    private void driveToGold(){
        double distance = DRIVE_TO_GOLD_DISTANCE;
        eReset();
        PIDController control = new PIDController(DRIVE_TO_GOLD_KP,DRIVE_TO_GOLD_KI,DRIVE_TO_GOLD_KD,1);
        double ticks = (distance/(WHEEL_DIAMETER*Math.PI))*DT_NEVEREST_GEARBOX;
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
            telemetry.update();

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            if (Math.abs(ticks-avg)<= distanceToTicks(DISTANCE_TOLERANCE)) {
                telemetry.addData("Distance from Target: ", Math.abs(ticks-avg));
                stopState = (System.nanoTime() - startTime) / 1000000;
            } else {
                startTime = System.nanoTime();
            }

            if(System.nanoTime()/1000000-beginTime/1000000>5500){
                break;
            }

        }
    }

    private void rotate90(){
        double degrees = ROTATE_90_ANGLE;
        PIDController controlRotate = new PIDController(ROTATE_90_KP,ROTATE_90_KI,ROTATE_90_KD,1);
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
            if(System.nanoTime()/1000000-beginTime/1000000>2000){
                break;
            }
        }
    }

    private void driveToSample2(){
        double distance = DRIVE_TO_SAMPLE_2_DISTANCE;
        eReset();
        PIDController control = new PIDController(DRIVE_TO_SAMPLE_2_KP,DRIVE_TO_SAMPLE_2_KI,DRIVE_TO_SAMPLE_2_KD,1);
        double ticks = (distance/(WHEEL_DIAMETER*Math.PI))*DT_NEVEREST_GEARBOX;
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
            telemetry.update();

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            if (Math.abs(ticks-avg)<= distanceToTicks(DISTANCE_TOLERANCE)) {
                telemetry.addData("Distance from Target: ", Math.abs(ticks-avg));
                stopState = (System.nanoTime() - startTime) / 1000000;
            } else {
                startTime = System.nanoTime();
            }

            if(System.nanoTime()/1000000-beginTime/1000000>5500){
                break;
            }

        }
    }

    private void rotateToDepot(){
        double degrees = ROTATE_TO_DEPOT_ANGLE;
        PIDController controlRotate = new PIDController(ROTATE_TO_DEPOT_KP,ROTATE_TO_DEPOT_KI,ROTATE_TO_DEPOT_KD,1);
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
            if(System.nanoTime()/1000000-beginTime/1000000>2000){
                break;
            }
        }
    }

    private void driveToDepot(){
        double distance = DRIVE_TO_DEPOT_DISTANCE;
        eReset();
        PIDController control = new PIDController(DRIVE_TO_DEPOT_KP,DRIVE_TO_DEPOT_KI,DRIVE_TO_DEPOT_KD,1);
        double ticks = (distance/(WHEEL_DIAMETER*Math.PI))*DT_NEVEREST_GEARBOX;
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
            telemetry.update();

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            if (Math.abs(ticks-avg)<= distanceToTicks(DISTANCE_TOLERANCE)) {
                telemetry.addData("Distance from Target: ", Math.abs(ticks-avg));
                stopState = (System.nanoTime() - startTime) / 1000000;
            } else {
                startTime = System.nanoTime();
            }

            if(System.nanoTime()/1000000-beginTime/1000000>5500){
                break;
            }

        }
    }

    private void driveBackToGold(){
        double distance = DRIVE_BACK_TO_GOLD_DISTANCE;
        eReset();
        PIDController control = new PIDController(DRIVE_BACK_TO_GOLD_KP,DRIVE_BACK_TO_GOLD_KI,DRIVE_BACK_TO_GOLD_KD,1);
        double ticks = (distance/(WHEEL_DIAMETER*Math.PI))*DT_NEVEREST_GEARBOX;
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
            telemetry.update();

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            if (Math.abs(ticks-avg)<= distanceToTicks(DISTANCE_TOLERANCE)) {
                telemetry.addData("Distance from Target: ", Math.abs(ticks-avg));
                stopState = (System.nanoTime() - startTime) / 1000000;
            } else {
                startTime = System.nanoTime();
            }

            if(System.nanoTime()/1000000-beginTime/1000000>5500){
                break;
            }

        }
    }

    private void rotateToGold2(){
        double degrees = ROTATE_TO_GOLD_2_ANGLE;
        PIDController controlRotate = new PIDController(ROTATE_TO_GOLD_2_KP,ROTATE_TO_GOLD_2_KI,ROTATE_TO_GOLD_2_KD,1);
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
            if(System.nanoTime()/1000000-beginTime/1000000>2000){
                break;
            }
        }
    }

    private void driveToGold2(){
        double distance = DRIVE_TO_GOLD_2_DISTANCE;
        eReset();
        PIDController control = new PIDController(DRIVE_TO_GOLD_2_KP,DRIVE_TO_GOLD_2_KI,DRIVE_TO_GOLD_2_KD,1);
        double ticks = (distance/(WHEEL_DIAMETER*Math.PI))*DT_NEVEREST_GEARBOX;
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
            telemetry.update();

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            if (Math.abs(ticks-avg)<= distanceToTicks(DISTANCE_TOLERANCE)) {
                telemetry.addData("Distance from Target: ", Math.abs(ticks-avg));
                stopState = (System.nanoTime() - startTime) / 1000000;
            } else {
                startTime = System.nanoTime();
            }

            if(System.nanoTime()/1000000-beginTime/1000000>5500){
                break;
            }

        }
    }

    public void stop(){
        for(SpeedControlledMotor motor: hardware.drivetrainMotors) {
            motor.setPower(0);
        }
    }

    private final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void eReset() {

        for(SpeedControlledMotor motor: hardware.drivetrainMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private double distanceToTicks(double distance){
        return (distance/(WHEEL_DIAMETER*Math.PI))*DT_NEVEREST_GEARBOX;
    }
    private double ticksToDistance(double ticks){
        return (ticks*(WHEEL_DIAMETER*Math.PI))/DT_NEVEREST_GEARBOX;
    }
    public boolean opModeIsActive() {
        return auto.getOpModeIsActive();
    }

}
