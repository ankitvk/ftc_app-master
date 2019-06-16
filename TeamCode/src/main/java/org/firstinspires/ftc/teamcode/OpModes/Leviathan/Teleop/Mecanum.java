package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@TeleOp(name="Opportunity")
public class Mecanum extends OpMode {

    public Hardware r = new Hardware();
    private String driveType = "UNKNOWN";

    @Override
    public void init() {
        r.opportunityInit(hardwareMap);
        r.setAuto(null,telemetry);
        r.imu.resetYaw();

        telemetry.addLine("Opportunity time");
        telemetry.addLine("Drive Type:" + driveType);
        telemetry.addData("imu:",r.imu.getYaw());
        telemetry.update();
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            r.drivetrain.mecanum(gamepad1);
            driveType = "NORMAL";
        }
        else if(gamepad1.b){
            r.drivetrain.fieldCentric(gamepad1);
            driveType = "FIELD-CENTRIC";
        }
        if(gamepad1.left_stick_button && gamepad1.right_stick_button) r.imu.resetYaw();

        telemetry.addLine("Drive Type:" + driveType);
        telemetry.addData("FL:",r.frontLeft.getPower());
        telemetry.addData("BL:",r.backLeft.getPower());
        telemetry.addData("FR:",r.frontRight.getPower());
        telemetry.addData("BR:",r.backRight.getPower());
        telemetry.addData("imu:",r.imu.getYaw());
        telemetry.update();
    }
}
