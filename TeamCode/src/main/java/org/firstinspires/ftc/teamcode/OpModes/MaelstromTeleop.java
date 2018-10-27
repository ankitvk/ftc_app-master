package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.PointF;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.PathFollower;

@TeleOp(name="TeleOp")

//@Disabled
public class MaelstromTeleop extends OpMode implements Constants{

    Hardware robot = new Hardware();
    PointF coordinate = new PointF(0,0);
    PathFollower tracker = new PathFollower();
    double yDirection;
    double xDirection;
    boolean extendHasStopped = false;



    public void init(){

        robot.init(hardwareMap);
    }

    public void loop(){

        yDirection = gamepad1.left_stick_y;
        xDirection = gamepad1.right_stick_x;

        robot.backLeft.setPower(yDirection+xDirection);
        robot.frontLeft.setPower(yDirection+xDirection);
        robot.backRight.setPower(xDirection-yDirection);
        robot.frontRight.setPower(xDirection-yDirection);

        if (yDirection>=0){
            robot.backLeft.setPower(yDirection+xDirection);
            robot.frontLeft.setPower(yDirection+xDirection);
            robot.backRight.setPower(xDirection-yDirection);
            robot.frontRight.setPower(xDirection-yDirection);
        }
        else {
            robot.backLeft.setPower(yDirection-xDirection);
            robot.frontLeft.setPower(yDirection-xDirection);
            robot.backRight.setPower(-xDirection-yDirection);
            robot.frontRight.setPower(-xDirection-yDirection);
        }

        if (gamepad2.right_trigger>0){
            robot.extensionRight.setPower(-gamepad2.right_trigger);
            robot.extensionLeft.setPower(-gamepad2.right_trigger);
        }
        else if (gamepad2.left_trigger>0){
            robot.extensionLeft.setPower(gamepad2.left_trigger);
            robot.extensionRight.setPower(gamepad2.left_trigger);
        }
        else if(extendHasStopped){
            extendHasStopped = !extendHasStopped;

        }
        else{
            extendHasStopped = true;
        }

        if (gamepad1.right_trigger>0){
            robot.pivot1.setPower(.75);
            robot.pivot2.setPower(.75);
        }
        else if




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