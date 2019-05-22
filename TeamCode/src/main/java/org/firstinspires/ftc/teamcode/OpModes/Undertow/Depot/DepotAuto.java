package org.firstinspires.ftc.teamcode.OpModes.Undertow.Depot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;

@Disabled
@Autonomous(name = "DepotAutoDoubleSample",group = "Depot")
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
        goldfish.setAlignSettings(Companion.getALIGN_POSITION(), 1000);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.init(hardwareMap);
        //start opencv
        double goldPos = 0;

        waitForStart();

        goldfish.startOpenCV(); //start opencv

        drivetrain.rotateForTime(-.5, 600);
        drivetrain.stop();

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

        double postSample = robot.getImu().getYaw();
        drivetrain.rotateToAbsoluteAngle(-postSample);

        sleep(3000);

        double distanceTwo;

        if(postSample>25){
             distanceTwo = -25;
            drivetrain.rotateToAbsoluteAngle(0);

            drivetrain.driveForwardDistance(12);

            drivetrain.rotateToAbsoluteAngle(-90);



            drivetrain.driveForwardDistance(distanceTwo);



            drivetrain.rotateToBigAbsoluteAngle(-180);

            drivetrain.driveForwardDistance(-30);
        }
        else if(postSample<-25){
             distanceTwo = -10;
            drivetrain.rotateToAbsoluteAngle(0);

            drivetrain.driveForwardDistance(12);

            drivetrain.rotateToAbsoluteAngle(-90);



            drivetrain.driveForwardDistance(distanceTwo);



            drivetrain.rotateToBigAbsoluteAngle(-180);

            drivetrain.driveForwardDistance(-55);

            drivetrain.rotateToAbsoluteAngle(-90);

            drivetrain.driveForwardDistance(-25);
        }
        else{
            distanceTwo = -18;
            drivetrain.rotateToAbsoluteAngle(0);

            drivetrain.driveForwardDistance(12);

            drivetrain.rotateToAbsoluteAngle(-90);



            drivetrain.driveForwardDistance(distanceTwo);



            drivetrain.rotateToBigAbsoluteAngle(-185);

            drivetrain.driveForwardDistance(-40);
        }

        drivetrain.rotateToAbsoluteAngle(-90);

        drivetrain.driveForwardDistance(-25);

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