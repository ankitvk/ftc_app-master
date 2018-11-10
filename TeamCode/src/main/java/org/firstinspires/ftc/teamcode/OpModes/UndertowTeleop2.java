package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@TeleOp(name="UndertowTeleOp")
public class UndertowTeleop2 extends OpMode implements Constants{

    private Hardware robot = new Hardware();
    private double yDirection;
    private double xDirection;

    public void init(){

        robot.init(hardwareMap);
        robot.marker.setPosition(.5);
    }

    public void loop(){

        //dt control
        yDirection = gamepad1.left_stick_y;
        xDirection = gamepad1.right_stick_x;

        /*if(gamepad1.x){
            driveFront = !driveFront;
        }

        if(driveFront){
            robot.backLeft.setPower(-yDirection-xDirection);
            robot.frontLeft.setPower(-yDirection-xDirection);
            robot.backRight.setPower(-xDirection+yDirection);
            robot.frontRight.setPower(-xDirection+yDirection);
        }
        else{*/
        robot.backLeft.setPower(yDirection+xDirection);
        robot.frontLeft.setPower(yDirection+xDirection);
        robot.backRight.setPower(xDirection-yDirection);
        robot.frontRight.setPower(xDirection-yDirection);
        //}

        if(gamepad2.right_trigger>0){
            robot.extendo.setPower(gamepad2.right_trigger);
        }
        else if(gamepad2.left_trigger>0){
            robot.extendo.setPower(-gamepad2.left_trigger);
        }
        else if(gamepad2.dpad_up){
            robot.extendo.setPower(.01);
        }
        else if(gamepad2.dpad_down){
            robot.extendo.setPower(-.01);
        }
        else{
            robot.extendo.setPower(0);
        }

        if(gamepad2.a){
            robot.hook.setPosition(1);
        }

        if(gamepad2.x){
            robot.hook.setPosition(0);
        }

        telemetry.addData("Absolute Angle:",robot.imu.getYaw());
        telemetry.addData("LeftSpeed:",robot.frontLeft.getPower());
        telemetry.addData("RightSpeed:",robot.frontRight.getPower());
        telemetry.addData("Extendo:",robot.extendo.getCurrentPosition());
        telemetry.update();


    } //Ends main loop
}

/*
                                                                                ``````````..`  ``````````
                                                                   ``..---::://///////////:.`-::::///////////::::---..```
                                                              `.-:://///:---...`````                    ``````....---::://::--..`
                                                            `:////:-.``                                                  ``..--://:--.`
                                                           .////:`                                                               `..-:/:-.`
                                                           -///:                                                                      `.-//-`
              `-.       --       --`      `--------`  `-.  `////.              ---------. .-------.    .--------`   `-`      `-`          -//-
               -/:`     //.      ://.      :/:::::::   :/`  `:///:.`           -/:://///:``:/:::::/:   -//:/://:/.   :/-     ./:           .//.
               `//:.   `///`     :/-/-     `/:         `/:    .:////-`             `/:     ./-    ./-  `/:      :/`  .//:`   -//-          .//-
                ./:/-  `/:/:     :/`./-     -/-````     -/.     .:////:.`           ./-     :/`    -/`  ./-     `/:   -/:/.  -/:/`        .//-
                 ::`::`./.-/.    -/` `/:`    :////:`     :/`      .://///-.          :/`    `/::::::/:   :/`     -/.  `:-./- :/`::    `.-::-`
         `.--..  `/- -/:/. ::    -/-..-/:`   `/:````     `/:        `-:////:-`       `/:     -/::://:-   `/:      ::`  ./``:::/ ./-  .```
       `:/-`      -/` .//` ./-   -////////.   -/.         -/.          .://///:.      -/.     :/` .::`    -/.     ./-   ::  -/:  :/`
      .//.         ::  `:`  -/`  ./-`````:/-   :/-------   ::------.     `-/////:.     ::`    `/-  `-/:`   ::------//`  `/-  .:  `/:
      :/:          `/.      `::  ./-      -/-  .////////.  .///////:        .:////:`   ./-     -/.   `:/-` `-////////.   -/`      ./.
      ://`          -:..............................................`         .:////.   `.``````..`````...```........`````..``````.::`
      `//:`          :///////////////////////////////////////////////`          :////`   :///////////////////////////////////////////-
       `://-`         ```````````````````````````````````````````````           .////.    ````````````````````````````````````````````
         .://:-`                                                               `:///:`        -:::::-`  .-:::::.     `-::.   `-::::-
            .-///:-.`                                                        `-////:`         .`  .//. .//-  :/:   `-:://`  -/:.`` `
               `.-:///:--.``                                             `.-:////:.            .::/:.  `://::/.  `-:. :/-  -//:::/:`
                    ``.--:////::---...``````                  ```...--:://///:-.`                `//: .//. `://`.//::://:. :/:  `//-
                            ``..---:::://///////- .::::::////////////:::--.`                ::---:/:` .//-.-:/- `````:/:`  -//-.:/-
                                         ``````..` `............```                         ``....`    `....`        ..`    `....`

 */