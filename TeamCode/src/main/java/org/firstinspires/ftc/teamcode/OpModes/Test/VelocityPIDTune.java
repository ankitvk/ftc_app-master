package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@TeleOp(name = "Velocity Tuning")
public class VelocityPIDTune extends OpMode implements Constants {
    private Hardware robot = new Hardware();

    double rightPower = .6;
    double leftPower = .6;
    double kp = 0, ki = 0, kd = 0;
    boolean setKp = false;

    public void init(){
        robot.opportunityInit(hardwareMap);
        kp = .0031;
        ki = robot.backRight.PIDController.getKi();
        kd = robot.backRight.PIDController.getKd();
        telemetry.addData("KP:",kp);
        telemetry.addData("KI:",ki);
        telemetry.addData("KD:",kd);
        telemetry.update();
    }

    public void loop(){

        if(gamepad1.dpad_right){
            leftPower+=.005;
            rightPower += .005;
        }
        else if(gamepad1.dpad_left){
            leftPower-=.005;
            rightPower -= .005;

        }

        if(gamepad1.a && setKp){
/*            robot.backRight.setPower(rightPower);
            robot.frontRight.setPower(rightPower);
            robot.backLeft.setPower(leftPower);*/
            setKp = false;
            robot.backRight.setSpeed(leftPower);
        }
        if(gamepad1.b){
/*            robot.backRight.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontLeft.setPower(0);*/
            robot.backRight.setPower(0);
        }
        if(gamepad1.dpad_up){
            kp += .001;
        }
        if(gamepad1.dpad_down) kp -= .001;

        if(gamepad1.y){
            robot.backRight.PIDController.setKp(kp);
            setKp = true;
        }

/*        telemetry.addData("backRightRPM:",robot.backRight.getVelocity());
        telemetry.addData("frontRightRPM:",robot.frontRight.getVelocity());
        telemetry.addData("backLeftRPM:",robot.backLeft.getVelocity());*/
        telemetry.addData("Motor Velocity:",robot.backRight.getVelocity());
        telemetry.addData("Motor Power:",robot.backRight.getPower());
        telemetry.addData("Motor RPM:",robot.backRight.getRPM());
        telemetry.addData("Target RPM: ", robot.backRight.getTargetVelocity(leftPower));
        //telemetry.addData("RightPower:",rightPower);
        telemetry.addData("LeftPower:",leftPower);
        telemetry.addData("KP:",kp);
/*        telemetry.addData("KI:",ki);
        telemetry.addData("KD:",kd);*/
        telemetry.addData("Set?:",setKp);
        telemetry.update();
    }

}
