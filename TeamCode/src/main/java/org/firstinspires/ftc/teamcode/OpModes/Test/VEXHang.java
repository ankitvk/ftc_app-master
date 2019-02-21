package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Drivetrain;

@TeleOp(name = "VEX Hang",group = "Test")
public class VEXHang extends LinearOpMode{

    CRServo hangLeftTop,hangLeftBottom,hangRightTop,hangRightBottom;

    Servo hangLeftRelease,hangRightRelease;

    @Override
    public void runOpMode(){

        hangLeftTop = hardwareMap.crservo.get("hangLeftTop");
        hangLeftBottom = hardwareMap.crservo.get("hangLeftBottom");
        hangRightTop = hardwareMap.crservo.get("hangRightTop");
        hangRightBottom = hardwareMap.crservo.get("hangRightBottom");

        hangRightRelease = hardwareMap.servo.get("hangRightRelease");
        hangLeftRelease = hardwareMap.servo.get("hangLeftRelease");

        hangLeftRelease.setPosition(.3);
        hangRightRelease.setPosition(.85);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                hangRightRelease.setPosition(.25);
                hangLeftRelease.setPosition(.75);
            }
            else{
                hangLeftRelease.setPosition(.3);
                hangRightRelease.setPosition(.85);
            }

            if(gamepad1.left_trigger>0){
                hangLeftBottom.setPower(.75);
                hangRightTop.setPower(.75);
                hangLeftTop.setPower(-.75);
                hangRightBottom.setPower(-.75);
            }
            else if(gamepad1.right_trigger>0){
                hangLeftBottom.setPower(-.75);
                hangRightTop.setPower(-.75);
                hangLeftTop.setPower(.75);
                hangRightBottom.setPower(.75);
            }
            else{
                hangLeftBottom.setPower(0);
                hangRightTop.setPower(0);
                hangLeftTop.setPower(0);
                hangRightBottom.setPower(0);
            }
        }

    }
}
