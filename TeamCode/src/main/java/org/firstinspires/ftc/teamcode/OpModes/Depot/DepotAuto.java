package org.firstinspires.ftc.teamcode.OpModes.Depot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;

@Autonomous(name = "DepotAuto",group = "Depot")
public class DepotAuto extends LinearOpMode implements AutonomousOpMode,Constants {

    Hardware robot = new Hardware();

    public boolean getOpModeIsActive() {
        return opModeIsActive();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    @Override
    public void runOpMode() {

        robot.setAuto(this, telemetry);

        GoldFind goldfish = new GoldFind(this,robot);
        goldfish.setAlignSettings(ALIGN_POSITION, 1000);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.init(hardwareMap);
        //start opencv
        double goldPos = 0;

        waitForStart();

        goldfish.startOpenCV(hardwareMap); //start opencv

        /*robot.marker.setPosition(.9);

        robot.hook.setPosition(.5);
        robot.extendo.setPower(1);
        sleep(1250);
        robot.extendo.setPower(0);
*/
        drivetrain.rotateForTime(-.5, 500);
        drivetrain.stop();

        /*robot.extendo.setPower(-1);
        sleep(1500);
        robot.extendo.setPower(0);
*/

        while(getOpModeIsActive() && !goldfish.getAligned()){
            drivetrain.rotate(-0.35);
            telemetry.addData("Aligned:",goldfish.getAligned());
            telemetry.addData("Pos:",goldfish.getXPosition());
            telemetry.update();
        }
        drivetrain.stop();
        goldfish.disable();

        drivetrain.driveForwardDistance(-25);
        drivetrain.stop();

        double postSample = robot.imu.getYaw();
        drivetrain.rotateToAbsoluteAngle(-postSample);

        sleep(5000);

        drivetrain.rotateToAbsoluteAngle(0);

        drivetrain.driveForwardDistance(12);

        drivetrain.rotateToAbsoluteAngle(-90);

        drivetrain.driveForwardDistance(-30);

        drivetrain.rotateToAbsoluteAngle(-180);



        //good till here

        /*double SamplePos = robot.imu.getYaw();
        if(SamplePos<-20){
            drivetrain.rotateToAbsoluteAngle(20);
            drivetrain.driveForwardDistance(-25);
            robot.marker.setPosition(.25);
            sleep(1000);
            robot.marker.setPosition(.75);
            drivetrain.rotateForTime(.5,750);
            drivetrain.rotateToAbsoluteAngle(-135);
            drivetrain.driveForwardDistance(-84);

            drivetrain.stop();

        }
        else if(SamplePos>20){
            drivetrain.rotateToAbsoluteAngle(-20);
            drivetrain.driveForwardDistance(-25);
            robot.marker.setPosition(.25);
            sleep(1000);
            robot.marker.setPosition(.75);
            *//*drivetrain.rotateForTime(-.5,750);
            drivetrain.rotateToAbsoluteAngle(135);
            drivetrain.driveForwardDistance(-84);*//*

            drivetrain.stop();
        }
        else{
            drivetrain.rotateToAbsoluteAngle(0);
            drivetrain.driveForwardDistance(-20);
            robot.marker.setPosition(.25);
            sleep(1000);
            robot.marker.setPosition(.75);
            drivetrain.rotateForTime(.5,750);
            drivetrain.rotateToAbsoluteAngle(-135);
            drivetrain.driveForwardDistance(-84);

            drivetrain.stop();
        }
*/


        drivetrain.stop();
    }//end opMode

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