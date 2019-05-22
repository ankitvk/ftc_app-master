package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@TeleOp(name = "Tune a bih")
public class VelocityPIDTune extends OpMode implements Constants {
    private Hardware robot = new Hardware();
    SpeedControlledMotor motor = robot.getBackLeft();

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
            robot.getBackRight().setPower(rightPower);
            robot.getFrontRight().setPower(rightPower);
            robot.getBackLeft().setPower(leftPower);
            robot.getFrontLeft().setPower(leftPower);
        }

        telemetry.addData("backRightRPM:", robot.getBackRight().getRPM());
        telemetry.addData("frontRightRPM:", robot.getFrontRight().getRPM());
        telemetry.addData("backLeftRPM:", robot.getBackLeft().getRPM());
        telemetry.addData("frontLeftRPM:", robot.getFrontLeft().getRPM());
        telemetry.addData("RightPower:",rightPower);
        telemetry.addData("LeftPower:",leftPower);


        telemetry.update();
    }

}
