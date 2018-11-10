package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;

@Autonomous(name = "DepotAuto")
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

        GoldFind goldfish = new GoldFind(this);
        goldfish.setAlignSettings(ALIGN_POSITION, 1000);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.init(hardwareMap);
        goldfish.startOpenCV(hardwareMap); //start opencv
        double goldPos = 0;

        waitForStart();

        //drivetrain.rotateToAbsoluteAngle(-40);

        drivetrain.rotateForTime(-.5,500);

        drivetrain.stop();

        /*drivetrain.rotate(-1);
        sleep(125);*/
        while(getOpModeIsActive() && !goldfish.getAligned()){
            drivetrain.rotate(-0.3);
            telemetry.addData("Aligned:",goldfish.getAligned());
            telemetry.addData("Pos:",goldfish.getXPosition());
            telemetry.update();
        }
        drivetrain.stop();
        goldfish.disable();

        drivetrain.driveForwardDistance(-25);

        //good till here

        /*double SamplePos = robot.imu.getYaw();
        if(SamplePos<-15){
            drivetrain.rotateToAbsoluteAngle(30);
            drivetrain.driveForwardDistance(20);

        }
        else if(SamplePos>15){
            drivetrain.rotateToAbsoluteAngle(-30);
            drivetrain.driveForwardDistance(20);
        }
        else{
            drivetrain.driveForwardDistance(15);
        }

        sleep(2000);*/

        //bad
        double SamplePos = robot.imu.getYaw();
        drivetrain.rotateToAbsoluteAngle(-SamplePos);
        //sleep(500);
        drivetrain.driveForwardDistance(-15);

        //sleep(3000);

        drivetrain.rotateForTime(.35,500);
        drivetrain.rotateToAbsoluteAngle(-135);
        drivetrain.driveForwardDistance(-84);

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