package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@TeleOp(name = "Tune a bih")
public class VelocityPIDTune extends OpMode implements Constants {
    private Hardware robot = new Hardware();
    SpeedControlledMotor motor = robot.backLeft;

    double rightPower = .6;
    double leftPower = .6;

    public void init(){
        robot.init(hardwareMap);
    }

    public void loop(){

        if(gamepad1.dpad_right){
            leftPower+=.0005;
            rightPower = leftPower+.1;
        }
        else if(gamepad1.dpad_left){
            leftPower-=.0005;
            rightPower = leftPower+.1;

        }

        if(gamepad1.a){
            robot.backRight.setPower(rightPower);
            robot.frontRight.setPower(rightPower);
            robot.backLeft.setPower(leftPower);
            robot.frontLeft.setPower(leftPower);
        }

        telemetry.addData("backRightRPM:",robot.backRight.getRPM());
        telemetry.addData("frontRightRPM:",robot.frontRight.getRPM());
        telemetry.addData("backLeftRPM:",robot.backLeft.getRPM());
        telemetry.addData("frontLeftRPM:",robot.frontLeft.getRPM());
        telemetry.addData("RightPower:",rightPower);
        telemetry.addData("LeftPower:",leftPower);


        telemetry.update();
    }

}
