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

    public void init(){
        robot.init(hardwareMap);
    }

    public void loop(){
        if(gamepad1.a){
            motor.setSpeed(0.9);
        }
        else{
            motor.setSpeed(0);
        }
        telemetry.addData("RPM",robot.backLeft.getRPM());

        if(gamepad1.dpad_up){
            motor = robot.backLeft;
            telemetry.addData("backLeft",robot.backLeft.getPower());
        }
        else if(gamepad1.dpad_right){
            motor = robot.frontLeft;
            telemetry.addData("frontLeft",robot.frontLeft.getPower());
        }
        else if(gamepad1.dpad_down){
            motor = robot.backRight;
            telemetry.addData("backRight",robot.backRight.getPower());

        }
        else if(gamepad1.dpad_left){
            motor = robot.frontRight;
            telemetry.addData("frontRight",robot.frontRight.getPower());

        }

        telemetry.update();
    }

}
