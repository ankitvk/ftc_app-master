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

        robot.drive.controlDrive(gamepad1);

        telemetry.addData("backRightRPM:",robot.backRight.getRPM());
        telemetry.addData("frontRightRPM:",robot.frontRight.getRPM());
        telemetry.addData("backLeftRPM:",robot.backLeft.getRPM());
        telemetry.addData("frontLeftRPM:",robot.frontLeft.getRPM());
        telemetry.addData("backRightPower:",robot.backRight.getPower());
        telemetry.addData("frontRightPower:",robot.frontRight.getPower());
        telemetry.addData("backLeftPower:",robot.backLeft.getPower());
        telemetry.addData("frontLeftPower:",robot.frontLeft.getPower());
        telemetry.addData("Forward speed:",gamepad1.left_stick_y);
        telemetry.addData("desired rpm:",robot.frontLeft.getRpmTemp());

        telemetry.update();
    }

}
