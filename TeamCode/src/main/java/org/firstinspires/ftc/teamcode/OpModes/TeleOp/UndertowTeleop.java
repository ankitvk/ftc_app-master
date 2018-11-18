package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@TeleOp(name="UndertowTeleOpOG")
public class UndertowTeleop extends OpMode implements Constants{

    private Hardware robot = new Hardware();
    private double yDirection;
    private double xDirection;

    public void init(){

        robot.init(hardwareMap);
        robot.hook.setPosition(0);
        //robot.marker.setPosition(.5);
    }

    public void loop(){

        //dt control
        yDirection = gamepad1.left_stick_y;
        xDirection = gamepad1.right_stick_x;

        robot.backLeft.setPower(-(yDirection+xDirection)*SPEED_MULTIPLIER);
        robot.frontLeft.setPower(-(yDirection+xDirection)*(SPEED_MULTIPLIER));
        robot.backRight.setPower(-(xDirection-yDirection)*(SPEED_MULTIPLIER));
        robot.frontRight.setPower(-(xDirection-yDirection)*(SPEED_MULTIPLIER));

        if(gamepad1.right_bumper){
            robot.extendo.setPower(1);
        }
        else if(gamepad1.left_bumper){
            robot.extendo.setPower(-1);
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

        if(gamepad2.y){
            robot.marker.setPosition(0);
        }
        if(gamepad2.b){
            robot.marker.setPosition(1);
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