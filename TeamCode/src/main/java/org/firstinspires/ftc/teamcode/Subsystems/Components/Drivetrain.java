package org.firstinspires.ftc.teamcode.Subsystems.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Drivers.BNO055_IMU;

import ftc.library.MaelUtils.MaelUtils;

public class Drivetrain implements Constants {

    private SpeedControlledMotor frontLeft,backLeft,frontRight,backRight;
    private BNO055_IMU imu;
    private AutonomousOpMode auto;
    private Telemetry telemetry;
    private Hardware hardware;
    private double desiredPitch = 0;
    public double speedMultipler = 0;
    public double turnMultipler = 0;

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
        PIDController control = new PIDController(.0007,dtKI,0,0);
        double ticks = (distance/(WHEEL_DIAMETER*Math.PI))*DT_GEARBOX_TICKS_PER_ROTATION;
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 62.5)){
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

            /*if(System.nanoTime()/1000000-beginTime/1000000>3000){
                break;
            }*/
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
        PIDController controlRotate = new PIDController(dtRotateKP,dtRotateKI,dtRotateKD,dtRotateMaxI); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive()/* && (stopState <= 1000)*/){
            double position = hardware.imu.getRelativeYaw();
            double power = controlRotate.power(degrees,position);
            /*if(Math.abs(power)<.3){
                if(Math.abs(power)<.01){
                    break;
                }
                else if(power>=0){
                    power = .3;
                }
                else {
                    power = -.3;
                }
            }*/
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
                /*stopState = (System.nanoTime() - startTime) / 1000000;*/
                break;
            }
            /*else {
                startTime = System.nanoTime();
            }*/
            if(System.nanoTime()/1000000-beginTime/1000000>1000){
                break;
            }
        }
        stop();
    }

    public void rotateToBigAbsoluteAngle(double desire){
        double degrees = desire;
        PIDController controlRotate = new PIDController(dtBigRotateKP,dtBigRotateKI,dtBigRotateKD,dtBigRotateMaxI);
        long startTime = System.nanoTime();
        long beginTime = startTime;

        long stopState = 0;
        while((opModeIsActive() && (stopState <= 1000))){
            double position = hardware.imu.getRelativeYaw();
            double power = controlRotate.power(degrees,position);
            /*if(Math.abs(power)<.35){
                if(Math.abs(power)<.01){
                    break;
                }
                else if(power>=0){
                    power = .35;
                }
                else {
                    power = -.35;
                }
            }*/
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
            /*if(System.nanoTime()/1000000-beginTime/1000000>1500){
                break;
            }*/
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

    private double distanceToTicks(double distance){
        return (distance/(WHEEL_DIAMETER*Math.PI))*DT_GEARBOX_TICKS_PER_ROTATION;
    }
    private double ticksToDistance(double ticks){
        return (ticks*(WHEEL_DIAMETER*Math.PI))/DT_GEARBOX_TICKS_PER_ROTATION;
    }

    public void rotateToRelativeAngle(double degrees){
        rotateToAbsoluteAngle(hardware.imu.getYaw()+degrees);
    }

    public void drive(Gamepad gamepad){
        PIDController pitchCorrection = new PIDController(.02,0,0,0);
        double antiTipPower = Math.abs(desiredPitch - imu.getPitch()) > 2?  pitchCorrection.power(desiredPitch, imu.getPitch()) : 0;
        double yDirection = gamepad.left_stick_y;
        double xDirection = gamepad.right_stick_x;
        double speedReducer = 1;

        double leftPower = (yDirection-xDirection)*(SPEED_MULTIPLIER);
        double rightPower = (-yDirection-xDirection)*(SPEED_MULTIPLIER);

        if(gamepad.a){
            speedReducer = 0.25;
        }
        else{
            speedReducer = 1;
        }
        hardware.backLeft.setPower(leftPower*speedReducer + antiTipPower);
        hardware.frontLeft.setPower(leftPower*speedReducer + antiTipPower);
        hardware.backRight.setPower(rightPower*speedReducer - antiTipPower);
        hardware.frontRight.setPower(rightPower*speedReducer - antiTipPower);
    }

    public void fieldCentric(Gamepad controller){

        double leftY = controller.left_stick_y;
        double leftX = controller.left_stick_x;
        double rightX = controller.right_stick_x;

        double x = -leftY;
        double y = leftX;

        double angle = Math.atan2(y,x);
        double fieldCentric = angle - Math.toRadians(imu.getYaw());
        double adjustedAngle = fieldCentric + Math.PI / 4;

        double speedMagnitude = Math.hypot(x,y);

        double speeds[] = {Math.sin(adjustedAngle), Math.cos(adjustedAngle), Math.cos(adjustedAngle), Math.sin(adjustedAngle)};

        MaelUtils.normalizeValues(speeds);

        speeds[0] = (speeds[0] * speedMagnitude * speedMultipler) - rightX * turnMultipler;
        speeds[1] = (speeds[1] * speedMagnitude * speedMultipler) - rightX * turnMultipler;
        speeds[2] = (speeds[2] * speedMagnitude * speedMultipler) + rightX * turnMultipler;
        speeds[3] = (speeds[3] * speedMagnitude * speedMultipler) + rightX * turnMultipler;

        frontLeft.setPower(speeds[0]);
        backLeft.setPower(speeds[1]);
        frontRight.setPower(speeds[2]);
        backRight.setPower(speeds[3]);

        if(controller.left_stick_button && controller.right_stick_button) imu.resetYaw();
    }


    public void mecanum(Gamepad gamepad){

        double leftY = gamepad.left_stick_y;
        double leftX = gamepad.left_stick_x;
        double rightX = gamepad.right_stick_x;
        double x = -leftY;
        double y = leftX;
        double angle = Math.atan2(y,x);
        double adjustedAngle = angle + Math.PI / 4;
        double speedMagnitude = Math.hypot(x,y);

        double speeds[] = {Math.sin(adjustedAngle), Math.cos(adjustedAngle), Math.cos(adjustedAngle), Math.sin(adjustedAngle)};

        MaelUtils.normalizeValues(speeds);

        speeds[0] = (speeds[0] * speedMagnitude * speedMultipler) - rightX * turnMultipler;
        speeds[1] = (speeds[1] * speedMagnitude * speedMultipler) - rightX * turnMultipler;
        speeds[2] = (speeds[2] * speedMagnitude * speedMultipler) + rightX * turnMultipler;
        speeds[3] = (speeds[3] * speedMagnitude * speedMultipler) + rightX * turnMultipler;

        frontLeft.setPower(speeds[0]);
        backLeft.setPower(speeds[1]);
        frontRight.setPower(speeds[2]);
        backRight.setPower(speeds[3]);

        telemetry.addData("FL:",frontLeft.getPower());
        telemetry.addData("BL:",backLeft.getPower());
        telemetry.addData("FR:",frontRight.getPower());
        telemetry.addData("BR:",backRight.getPower());
        telemetry.addData("imu:",imu.getYaw());
        telemetry.update();

    }
    public void resetDesiredPitch() {
        desiredPitch = imu.getPitch();
    }

    public void controlDrive(Gamepad gamepad){
        double yDirection = gamepad.left_stick_y;
        double xDirection = gamepad.right_stick_x;

        double leftPower = (yDirection-xDirection)*(SPEED_MULTIPLIER);
        double rightPower = (-yDirection-xDirection)*(SPEED_MULTIPLIER);
        hardware.frontLeft.setSpeed(leftPower);
        hardware.backLeft.setPower(hardware.frontLeft.getPower());
        hardware.frontRight.setSpeed(rightPower);
        hardware.backRight.setPower(hardware.frontRight.getPower());

    }

    public void driveForwardDistancePOM(double distance){
        eReset();
        PIDController control = new PIDController(-0.0001,0.000001,0,1);
        double ticks = (distance/(WHEEL_DIAMETER*Math.PI))*DT_GEARBOX_TICKS_PER_ROTATION;
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 250)){
            double avg = hardware.frontLeft.getCurrentPosition();
            double power = control.pom(ticks,avg,ticks);
            telemetry.addData("Power: ", power);
            telemetry.addData("Distance: ",ticksToDistance(avg));
            telemetry.addData("Angle: ", hardware.imu.getYaw());
            telemetry.addLine(" ");
            telemetry.addData("error: ",control.getError());
            telemetry.addData("KP*deltaDistance: ",control.getKp()*ticks);
            telemetry.addData("KI*i: ",control.returnVal()[1]);
            telemetry.addData("KD*d: ",control.returnVal()[2]);
            telemetry.update();

            hardware.frontLeft.setSpeed(-power);
            hardware.backLeft.setPower(hardware.frontLeft.getPower());
            hardware.frontRight.setSpeed(power);
            hardware.backRight.setPower(hardware.frontRight.getPower());

            if (Math.abs(ticks-avg)<= distanceToTicks(DISTANCE_TOLERANCE)) {
                telemetry.addData("Distance from Target: ", Math.abs(ticks-avg));
                stopState = (System.nanoTime() - startTime) / 1000000;
            } else {
                startTime = System.nanoTime();
            }
        }
    }


}
