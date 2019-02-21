package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Teleop;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Drivers.Music;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Endgame;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Extendo;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Hang;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Pivot;

@TeleOp(name="LeviathanTeleop")
public class LeviathanTeleop extends OpMode implements Constants{

    private Hardware robot = new Hardware();

    private Pivot pivot = new Pivot(robot);
    private Extendo extendo = new Extendo(robot);
    private Intake intake =  new Intake(robot);
    private Drivetrain drivetrain = new Drivetrain(robot);
    private Endgame endgame = new Endgame(robot);
    private Hang hang = new Hang(robot);

    private Music music;

    public boolean playing;

    private boolean resetimu = true;


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
        drivetrain.drive(gamepad1);
        pivot.driverControl(gamepad2);
        extendo.driverControl(gamepad1);
        intake.intake(gamepad1);
        intake.index(gamepad2);
        hang.lift(gamepad2);


        telemetry.addData("pivot1",robot.pivot1.getCurrentPosition());
        telemetry.addData("pivot2",robot.pivot2.getCurrentPosition());
        telemetry.addData("winch?",robot.winch.getCurrentPosition());
        telemetry.addData("extendo",robot.extendo.getCurrentPosition());

        telemetry.update();

    }
}