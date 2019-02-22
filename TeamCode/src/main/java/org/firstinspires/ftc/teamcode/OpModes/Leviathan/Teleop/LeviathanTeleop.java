package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.Toggle;
import org.firstinspires.ftc.teamcode.Drivers.Music;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@TeleOp(name="LeviathanTeleop")
public class LeviathanTeleop extends OpMode implements Constants{

    private Hardware robot = new Hardware();

    private Music music;

    public boolean playing;

    private boolean resetimu = true;

    private Toggle x = new Toggle();

    public void init(){
        robot.init(hardwareMap);

        music = new Music(robot.getHwMap().appContext);

        playing = true;

    }
    public void loop(){

        if(playing){
            music.start(Music.Songs.SEA_SHANTY_2);
            playing = !playing;
        }

        robot.drivetrain.drive(gamepad1);
        robot.pivot.driverControl(gamepad2);
        robot.extendo.driverControl(gamepad1);
        robot.intake.intake(gamepad1);
        robot.intake.index(gamepad2);
        robot.hang.lift(gamepad2);

        if (x.toggle(gamepad2.x)) {
            robot.hangLeftRelease.setPosition(.75);
            robot.hangRightRelease.setPosition(.25);
        } else {
            robot.hangLeftRelease.setPosition(.3);
            robot.hangRightRelease.setPosition(.85);
        }

        telemetry.addData("pivot1",robot.pivot1.getCurrentPosition());
        telemetry.addData("pivot2",robot.pivot2.getCurrentPosition());
        telemetry.addData("winch?",robot.winch.getCurrentPosition());
        telemetry.addData("extend",robot.extend.getCurrentPosition());
        telemetry.addData("pivot angle:", robot.pivot.getAngle());
        telemetry.update();

        telemetry.update();

    }
}