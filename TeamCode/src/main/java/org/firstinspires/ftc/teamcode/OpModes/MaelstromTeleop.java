package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.PointF;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;
import org.firstinspires.ftc.teamcode.Subsystems.PathFollower;

@TeleOp(name="TeleOp")

//@Disabled
public class MaelstromTeleop extends OpMode implements Constants{

    private Hardware robot = new Hardware();
    private double yDirection;
    private double xDirection;
    GoldFind goldfish = new GoldFind();


    private PIDController controlExtend = new PIDController(extensionKP,extensionKI,extensionKD,extensionMaxI);
    private PIDController controlPivot = new PIDController(pivotKP,pivotKI,pivotKD,pivotMaxI);

    double holdExtendPos = 0;

    private double holdPivotPos = 0;
    private double pivotPower = 0;

    private boolean intakeBoolCurr;
    private boolean intakeBoolPrev;
    private boolean intakeOn;


    public void init(){

        robot.init(hardwareMap);
        goldfish.startOpenCV(hardwareMap);
    }

    public void loop(){

        //dt control
        yDirection = gamepad1.left_stick_y;
        xDirection = gamepad1.right_stick_x;

        robot.backLeft.setPower(-yDirection-xDirection);
        robot.frontLeft.setPower(-yDirection-xDirection);
        robot.backRight.setPower(-xDirection+yDirection);
        robot.frontRight.setPower(-xDirection+yDirection);


        //extension control
        if (gamepad1.right_trigger>0){
            robot.extensionRight.setPower(gamepad1.right_trigger);
            robot.extensionLeft.setPower(gamepad1.right_trigger);
            holdExtendPos = robot.extensionRight.getCurrentPosition();
        }
        else if (gamepad1.left_trigger>0){
            robot.extensionLeft.setPower(-gamepad1.left_trigger);
            robot.extensionRight.setPower(-gamepad1.left_trigger);
            holdExtendPos = robot.extensionRight.getCurrentPosition();
        }
        else {
            robot.extensionLeft.setPower(0);
            robot.extensionRight.setPower(0);
        }

        //pivot control
        if(gamepad1.right_bumper){
            robot.pivot1.setPower(.75);
            robot.pivot2.setPower(.75);
            holdPivotPos = robot.pivot1.getCurrentPosition();
        }
        else if (gamepad1.left_bumper){
            robot.pivot1.setPower(-.15);
            robot.pivot2.setPower(-.15);
            holdPivotPos = robot.pivot1.getCurrentPosition();
        }
        /*else if(!pivotHasStopped){
            pivotHasStopped = true;
            holdPivotPos = robot.pivot1.getCurrentPosition();
        }*/
        else {
            //pivotPower = controlPivot.power(holdPivotPos,robot.pivot1.getCurrentPosition());
            robot.pivot1.setPower(0);
            robot.pivot2.setPower(0);
        }

        //intake d1
        if (gamepad1.x && gamepad1.start) {

            intakeBoolCurr = true;

        }
        else {

            intakeBoolCurr = false;

            if (intakeBoolPrev) {

                intakeOn = !intakeOn;

            }
        }

        intakeBoolPrev = intakeBoolCurr;

        if (intakeOn) {

            robot.intake.setPower(1);

        } else {

            robot.intake.setPower(0);

        }

        //intake d2
        if (gamepad2.right_trigger>0){
            robot.intake.setPower(.4);
        }
        else if (gamepad2.left_trigger>0){
            robot.intake.setPower(-.4);
        }
        else{
            robot.intake.setPower(0);
        }

        //index
        if (gamepad2.right_bumper){
            robot.indexer.setPosition(robot.indexer.getPosition()+.05);
        }
        else if (gamepad2.left_bumper){
            robot.indexer.setPosition(robot.indexer.getPosition()-.05);
        }

        telemetry.addData("IndexVal:",robot.indexer.getPosition());
        telemetry.addData("Aligned?:",goldfish.getAligned());
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