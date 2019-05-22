package org.firstinspires.ftc.teamcode.Subsystems.Paths.Depot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Drivers.BNO055_IMU;

public class depotMiddle implements Constants {

    private SpeedControlledMotor frontLeft,backLeft,frontRight,backRight;
    private BNO055_IMU imu;
    private AutonomousOpMode auto;
    private Telemetry telemetry;
    private Hardware hardware;

    private final double ROTATE_TO_GOLD_ANGLE = -85;
    private final double ROTATE_TO_GOLD_KP = 0.015/*.02725*/;
    private final double ROTATE_TO_GOLD_KI = 1.5;
    private final double ROTATE_TO_GOLD_KD = 0;

    private final double DRIVE_TO_DEPOT_DISTANCE = 57.5;
    private final double DRIVE_TO_DEPOT_KP = .00025;
    private final double DRIVE_TO_DEPOT_KI = 0;
    private final double DRIVE_TO_DEPOT_KD = 0;

    private final double ROTATE_TO_CRATER_ANGLE = -48;
    private final double ROTATE_TO_CRATER_KP = .02;
    private final double ROTATE_TO_CRATER_KI = 0;
    private final double ROTATE_TO_CRATER_KD = 0;

    private final double DRIVE_TO_CRATER_DISTANCE = -70;
    private final double DRIVE_TO_CRATER_KP = .000325;
    private final double DRIVE_TO_CRATER_KI = 0/*0.01*/;
    private final double DRIVE_TO_CRATER_KD = 0;

    public depotMiddle(Hardware hardware){
        this.hardware = hardware;
        frontLeft = hardware.getFrontLeft();
        backLeft = hardware.getBackLeft();
        frontRight = hardware.getFrontRight();
        backRight = hardware.getBackRight();
        imu = hardware.getImu();
        auto = hardware.getAuto();
        telemetry = hardware.getTelemetry();
    }

    public void run(){
        driveToDepot();
        rotateToCrater();
        marker();
        driveToCrater();
    }

    private void rotateToGold(){
        double degrees = ROTATE_TO_GOLD_ANGLE;
        PIDController controlRotate = new PIDController(ROTATE_TO_GOLD_KP,ROTATE_TO_GOLD_KI,ROTATE_TO_GOLD_KD,1);
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 250)){

            double position = hardware.getImu().getRelativeYaw();
            double power = controlRotate.power(degrees,position);

            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", hardware.getImu().getRelativeYaw());
            telemetry.addLine(" ");
            telemetry.addData("error: ",controlRotate.getError());
            telemetry.addData("KP*error: ",controlRotate.returnVal()[0]);
            telemetry.addData("KI*i: ",controlRotate.returnVal()[1]);
            telemetry.update();

            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);

            if (Math.abs(position-degrees) <= Companion.getIMU_TOLERANCE()) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
            /*if(System.nanoTime()/1000000-beginTime/1000000>5000){
                break;
            }*/
        }
    }

    private void driveToDepot(){
        double distance = DRIVE_TO_DEPOT_DISTANCE;
        eReset();
        PIDController control = new PIDController(DRIVE_TO_DEPOT_KP,DRIVE_TO_DEPOT_KI,DRIVE_TO_DEPOT_KD,1);
        double ticks = (distance/(Companion.getWHEEL_DIAMETER() *Math.PI))* Companion.getDT_GEARBOX_TICKS_PER_ROTATION();
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125)){
            double avg = hardware.getFrontLeft().getCurrentPosition();
            double power = control.power(ticks,avg);
            telemetry.addData("Power: ", power);
            telemetry.addData("Distance: ",ticksToDistance(avg));
            telemetry.addData("Angle: ", hardware.getImu().getYaw());
            telemetry.addData("error: ",control.getError());
            telemetry.addData("KP*error: ",control.returnVal()[0]);
            telemetry.addData("KI*i: ",control.returnVal()[1]);
            telemetry.addData("KD*d: ",control.returnVal()[2]);
            telemetry.update();

            hardware.getFrontLeft().setPower(-power);
            hardware.getBackLeft().setPower(-power);
            hardware.getFrontRight().setPower(power);
            hardware.getBackRight().setPower(power);

            if (Math.abs(ticks-avg)<= distanceToTicks(Companion.getDISTANCE_TOLERANCE())) {
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

            double position = hardware.getImu().getRelativeYaw();
            double power = controlRotate.power(degrees,position);

            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", hardware.getImu().getRelativeYaw());
            telemetry.addLine(" ");
            telemetry.addData("error: ",controlRotate.getError());
            telemetry.addData("KP*error: ",controlRotate.returnVal()[0]);
            telemetry.addData("KI*i: ",controlRotate.returnVal()[1]);
            telemetry.addData("KD*d: ",controlRotate.returnVal()[2]);
            telemetry.update();

            hardware.getFrontLeft().setPower(power);
            hardware.getBackLeft().setPower(power);
            hardware.getFrontRight().setPower(power);
            hardware.getBackRight().setPower(power);

            if (Math.abs(position-degrees) <= Companion.getIMU_TOLERANCE()) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
            if(System.nanoTime()/1000000-beginTime/1000000>3000){
                break;
            }
        }
    }

    private void driveToCrater(){
        double distance = DRIVE_TO_CRATER_DISTANCE;
        eReset();
        PIDController control = new PIDController(DRIVE_TO_CRATER_KP,DRIVE_TO_CRATER_KI,DRIVE_TO_CRATER_KD,1);
        double ticks = (distance/(Companion.getWHEEL_DIAMETER() *Math.PI))* Companion.getDT_GEARBOX_TICKS_PER_ROTATION();
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 1000)){
            double avg = hardware.getFrontLeft().getCurrentPosition();
            double power = control.power(ticks,avg);
            telemetry.addData("Power: ", power);
            telemetry.addData("Distance: ",ticksToDistance(avg));
            telemetry.addData("Angle: ", hardware.getImu().getYaw());
            telemetry.addLine(" ");
            telemetry.addData("error: ",control.getError());
            telemetry.addData("KP*error: ",control.returnVal()[0]);
            telemetry.addData("KI*i: ",control.returnVal()[1]);
            telemetry.addData("KD*d: ",control.returnVal()[2]);
            telemetry.update();

            hardware.getFrontLeft().setPower(-power);
            hardware.getBackLeft().setPower(-power);
            hardware.getFrontRight().setPower(power);
            hardware.getBackRight().setPower(power);

            if (Math.abs(ticks-avg)<= distanceToTicks(Companion.getDISTANCE_TOLERANCE())) {
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

    private void marker(){
        hardware.getWinch().setPower(-.35);
        sleep(2000);
        hardware.getWinch().setPower(0);
    }

    public void stop(){
        for(SpeedControlledMotor motor: hardware.getDrivetrainMotors()) {
            motor.setPower(0);
        }
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void eReset() {

        for(SpeedControlledMotor motor: hardware.getDrivetrainMotors()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private double distanceToTicks(double distance){
        return (distance/(Companion.getWHEEL_DIAMETER() *Math.PI))* Companion.getDT_GEARBOX_TICKS_PER_ROTATION();
    }
    private double ticksToDistance(double ticks){
        return (ticks*(Companion.getWHEEL_DIAMETER() *Math.PI))/ Companion.getDT_GEARBOX_TICKS_PER_ROTATION();
    }
    public boolean opModeIsActive() {
        return auto.getOpModeIsActive();
    }
}
