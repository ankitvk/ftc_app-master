package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.PointF;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
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


    PIDController controlExtend = new PIDController(extensionKP,extensionKI,extensionKD,extensionMaxI);
    PIDController controlPivot = new PIDController(pivotKP,pivotKI,pivotKD,pivotMaxI);

    boolean extendHasStopped = false;
    double holdExtendPos = 0;
    double extendPower;

    boolean pivotHasStopped = false;
    double holdPivotPos = 0;
    double pivotPower = 0;

    boolean intakeBoolCurr;
    boolean intakeBoolPrev;
    boolean intakeOn;



    public void init(){

        robot.init(hardwareMap);
    }

    public void loop(){

        //dt control
        yDirection = gamepad1.left_stick_y;
        xDirection = gamepad1.right_stick_x;

        /*robot.backLeft.setPower(yDirection+xDirection);
        robot.frontLeft.setPower(yDirection+xDirection);
        robot.backRight.setPower(xDirection-yDirection);
        robot.frontRight.setPower(xDirection-yDirection);*/

        robot.backLeft.setPower(-yDirection-xDirection);
        robot.frontLeft.setPower(-yDirection-xDirection);
        robot.backRight.setPower(-xDirection+yDirection);
        robot.frontRight.setPower(-xDirection+yDirection);


        //extension control
        if (gamepad1.right_trigger>0){
            robot.extensionRight.setPower(gamepad1.right_trigger);
            robot.extensionLeft.setPower(gamepad1.right_trigger);
            extendHasStopped = false;
        }
        else if (gamepad1.left_trigger>0){
            robot.extensionLeft.setPower(-gamepad1.left_trigger);
            robot.extensionRight.setPower(-gamepad1.left_trigger);
            extendHasStopped = false;
        } else {
            robot.extensionLeft.setPower(0);
            robot.extensionRight.setPower(0);
        }
/*        else if(!extendHasStopped){
            extendHasStopped = true;
            holdExtendPos = robot.extensionRight.getCurrentPosition();
        }
        else {
            extendPower = control.power(holdExtendPos,robot.extensionRight.getCurrentPosition());
            robot.extensionLeft.setPower(extendPower);
            robot.extensionRight.setPower(extendPower);
        }*/

        //pivot control
        if(gamepad1.right_bumper){
            robot.pivot1.setPower(.75);
            robot.pivot2.setPower(.75);
            pivotHasStopped = false;
        }
        else if (gamepad1.left_bumper){
            robot.pivot1.setPower(-.15);
            robot.pivot2.setPower(-.15);
            pivotHasStopped = false;
        }
        else if(!pivotHasStopped){
            pivotHasStopped = true;
            holdPivotPos = robot.pivot1.getCurrentPosition();
        }
        else
            {
                pivotPower = controlPivot.power(holdPivotPos,robot.pivot1.getCurrentPosition());
                robot.pivot1.setPower(pivotPower);
                robot.pivot2.setPower(pivotPower);
        }


        //pivot reset control
        /*if(robot.magLimitSwitch.getState()){
            robot.pivot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.pivot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }*/

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
            robot.intake.setPower(.5);
        }
        else if (gamepad2.left_trigger>0){
            robot.intake.setPower(-.5);
        }
        else{
            robot.intake.setPower(0);
        }

        //index
        if (gamepad2.x){
            robot.indexer.setPosition(1);
        }
        else if (gamepad2.a){
            robot.indexer.setPosition(0);
        }

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