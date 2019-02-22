package org.firstinspires.ftc.teamcode.Subsystems.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class Hang implements Constants {

    Hardware hardware;

    private AutonomousOpMode auto;
    private Telemetry telemetry;

    private CRServo hangLeftTop,hangLeftBottom,hangRightTop,hangRightBottom;

    private Servo hangLeftRelease,hangRightRelease;

    public Hang(Hardware hardware){
        this.hardware = hardware;

        hangLeftTop = hardware.hangLeftTop;
        hangLeftBottom = hardware.hangLeftBottom;
        hangRightTop = hardware.hangRightTop;
        hangRightBottom = hardware.hangRightBottom;

        hangLeftRelease = hardware.hangLeftRelease;
        hangRightRelease = hardware.hangRightRelease;

        auto = hardware.auto;
        telemetry = hardware.telemetry;

    }

    public void lift(Gamepad gamepad){

        double power;

        if(gamepad.right_trigger>0){
            power = gamepad.right_trigger;
        }
        else if (gamepad.left_trigger>0){
            power = -gamepad.left_trigger;
        }
        else{
            power = 0;
        }

        hardware.hangRightTop.setPower(.85*power);
        hardware.hangRightBottom.setPower(-.85*power);

        hardware.hangLeftTop.setPower(-.85*power);
        hardware.hangLeftBottom.setPower(.85*power);
    }

    public void drop(){
        hangLeftBottom.setPower(-.85);
        hangRightTop.setPower(-.85);
        hangLeftTop.setPower(.85);
        hangRightBottom.setPower(.85);

        sleep(250);

        hangLeftRelease.setPosition(.75);
        hangRightRelease.setPosition(.25);

        long startTime = System.nanoTime();
        long stopState = 0;
        while((stopState <= 2500)&&(opModeIsActive())){
            hangLeftBottom.setPower(.85);
            hangRightTop.setPower(.85);
            hangLeftTop.setPower(-.85);
            hangRightBottom.setPower(-.85);

            stopState = (System.nanoTime() - startTime) / 1000000;
        }
        stop();
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void stop(){
        hangLeftBottom.setPower(0);
        hangRightTop.setPower(0);
        hangLeftTop.setPower(0);
        hangRightBottom.setPower(0);
    }

    public boolean opModeIsActive() {
        return auto.getOpModeIsActive();
    }



}
