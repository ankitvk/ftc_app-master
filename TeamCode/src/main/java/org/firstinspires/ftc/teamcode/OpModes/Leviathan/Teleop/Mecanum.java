package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@TeleOp(name="Opportunity")
public class Mecanum extends OpMode {

    public Hardware r = new Hardware();

    @Override
    public void init() {
        r.opportunityInit(hardwareMap);
        r.setAuto(null,telemetry);
        r.getImu().resetYaw();

        telemetry.addLine("OPPORTUNITY");
        telemetry.addData("imu:", r.getImu().getYaw());
        telemetry.update();
    }

    @Override
    public void loop() {
        r.getDrivetrain().mecanum(gamepad1);

        if(gamepad1.left_stick_button && gamepad1.right_stick_button) r.getImu().resetYaw();
    }
}
