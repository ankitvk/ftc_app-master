package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

import ftc.library.MaelUtils.MaelUtils;

@TeleOp(name="Opportunity Normal")
public class MecanumFIeld extends LinearOpMode implements Constants {

    public Hardware r = new Hardware();
    public double speedMultipler = .85;
    public double turnMultipler = .85;
    //protected String driveType = "UNKNOWN";

/*    @Override
    public void runOpMode() throws InterruptedException {


        *//*while (!opModeIsActive()) {
*//**//*            telemetry.addLine("Opportunity time");
            //telemetry.addLine("Drive Type:" + driveType);
            telemetry.addData("FL:", r.frontLeft.getPower());
            telemetry.addData("BL:", r.backLeft.getPower());
            telemetry.addData("FR:", r.frontRight.getPower());
            telemetry.addData("BR:", r.backRight.getPower());
            telemetry.addData("imu:", r.imu.getYaw());
            telemetry.addData("turn speed:", r.drivetrain.turnMultipler);
            telemetry.addData("drive speed:", r.drivetrain.speedMultipler);
            telemetry.update();*//**//*
        }*//*



            if (gamepad1.left_stick_button && gamepad1.right_stick_button) r.imu.resetYaw();

*//*            if (gamepad1.dpad_up) r.drivetrain.speedMultipler += .01;
            else if (gamepad1.dpad_down) r.drivetrain.speedMultipler -= .01;

            if (gamepad1.dpad_left) r.drivetrain.turnMultipler -= .01;
            else if (gamepad1.dpad_right) r.drivetrain.turnMultipler += .01;

            r.drivetrain.turnMultipler = MaelUtils.clipValueToRange(r.drivetrain.turnMultipler, 0, 1);
            r.drivetrain.speedMultipler = MaelUtils.clipValueToRange(r.drivetrain.speedMultipler, 0, 1);
*//*
            //telemetry.addData("Drive Type: ", driveType);
*//*            telemetry.addData("FL: ", r.frontLeft.getPower());
            telemetry.addData("BL: ", r.backLeft.getPower());
            telemetry.addData("FR: ", r.frontRight.getPower());
            telemetry.addData("BR: ", r.backRight.getPower());
            telemetry.addData("imu: ", r.imu.getYaw());
            telemetry.addData("turn speed: ", r.drivetrain.turnMultipler);
            telemetry.addData("drive speed: ", r.drivetrain.speedMultipler);
            telemetry.update();*//*

    }*/

    @Override
    public void runOpMode() throws InterruptedException {
        r.opportunityInit(hardwareMap);
        r.imu.resetYaw();
        waitForStart();
        r.drivetrain.telemetry = this.telemetry;


        while (!isStopRequested()) {
            //r.drivetrain.mecanum(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            double leftY = gamepad1.left_stick_y;
            double leftX = gamepad1.left_stick_x;
            double rightX = gamepad1.right_stick_x;
            double x = leftY;
            double y = -leftX;
            double angle = Math.atan2(y,x);
            double adjustedAngle = angle + Math.PI / 4;
            double speedMagnitude = Math.hypot(x,y);

            double speeds[] = {Math.sin(adjustedAngle), Math.cos(adjustedAngle), Math.cos(adjustedAngle), Math.sin(adjustedAngle)};

            MaelUtils.normalizeValues(speeds);

            speeds[0] = (speeds[0] * speedMagnitude * speedMultipler) - rightX * turnMultipler;
            speeds[1] = (speeds[1] * speedMagnitude * speedMultipler) - rightX * turnMultipler;
            speeds[2] = (speeds[2] * speedMagnitude * speedMultipler) + rightX * turnMultipler;
            speeds[3] = (speeds[3] * speedMagnitude * speedMultipler) + rightX * turnMultipler;

            r.frontLeft.setPower(speeds[0]);
            r.backLeft.setPower(speeds[1]);
            r.frontRight.setPower(speeds[2]);
            r.backRight.setPower(speeds[3]);

            telemetry.addData("FL: ", r.frontLeft.getPower());
            telemetry.addData("BL: ", r.backLeft.getPower());
            telemetry.addData("FR: ", r.frontRight.getPower());
            telemetry.addData("BR: ", r.backRight.getPower());
            telemetry.addData("imu: ", r.imu.getYaw());
            telemetry.update();



        }
    }

}