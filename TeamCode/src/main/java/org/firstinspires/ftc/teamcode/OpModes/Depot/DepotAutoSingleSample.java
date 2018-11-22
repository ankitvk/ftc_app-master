package org.firstinspires.ftc.teamcode.OpModes.Depot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;

@Autonomous(name = "DepotAutoSingleSample",group = "Depot")
public class DepotAutoSingleSample extends LinearOpMode implements AutonomousOpMode,Constants {

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
        drivetrain.rotateForTime(-.5, 600);
        drivetrain.stop();

        /*robot.extendo.setPower(-1);
        sleep(1500);
        robot.extendo.setPower(0);
*/

        while(getOpModeIsActive() && !goldfish.getAligned()){
            drivetrain.rotate(-0.33);
            telemetry.addData("Aligned:",goldfish.getAligned());
            telemetry.addData("Pos:",goldfish.getXPosition());
            telemetry.update();
        }
        drivetrain.stop();
        goldfish.disable();

        drivetrain.driveForwardDistance(-25);
        drivetrain.stop();

        double postSample = robot.imu.getYaw();


        double distanceTwo;

        if(postSample>25){

            drivetrain.rotateToAbsoluteAngle(-10);

            robot.markerExtend1.setPower(1);
            robot.markerExtend2.setPower(1);

            sleep(1500);

            robot.markerExtend1.setPower(0);
            robot.markerExtend2.setPower(0);

            robot.marker.setPosition(1);

            robot.markerExtend1.setPower(-1);
            robot.markerExtend2.setPower(-1);

            sleep(1500);

            robot.markerExtend1.setPower(0);
            robot.markerExtend2.setPower(0);

            robot.marker.setPosition(0);
            distanceTwo = -45;
            drivetrain.rotateToAbsoluteAngle(0);

            drivetrain.driveForwardDistance(10);

            drivetrain.rotateToBigAbsoluteAngle(-87);



            drivetrain.driveForwardDistance(distanceTwo);

        }
        else if(postSample<-25){
            drivetrain.rotateToAbsoluteAngle(10);

            robot.markerExtend1.setPower(1);
            robot.markerExtend2.setPower(1);

            sleep(1500);

            robot.markerExtend1.setPower(0);
            robot.markerExtend2.setPower(0);

            robot.marker.setPosition(1);

            robot.markerExtend1.setPower(-1);
            robot.markerExtend2.setPower(-1);

            sleep(1500);

            robot.markerExtend1.setPower(0);
            robot.markerExtend2.setPower(0);

            robot.marker.setPosition(0);
            distanceTwo = -25;
            drivetrain.rotateToAbsoluteAngle(0);

            drivetrain.driveForwardDistance(10);

            drivetrain.rotateToBigAbsoluteAngle(-100);



            drivetrain.driveForwardDistance(distanceTwo);

        }
        else{
            drivetrain.rotateToAbsoluteAngle(0);

            robot.markerExtend1.setPower(1);
            robot.markerExtend2.setPower(1);

            sleep(1500);

            robot.markerExtend1.setPower(0);
            robot.markerExtend2.setPower(0);

            robot.marker.setPosition(1);

            robot.markerExtend1.setPower(-1);
            robot.markerExtend2.setPower(-1);

            sleep(1500);

            robot.markerExtend1.setPower(0);
            robot.markerExtend2.setPower(0);

            robot.marker.setPosition(0);
            distanceTwo = -30;

            drivetrain.rotateToAbsoluteAngle(0);

            drivetrain.driveForwardDistance(10);

            drivetrain.rotateToBigAbsoluteAngle(-90);



            drivetrain.driveForwardDistance(distanceTwo);

        }

        drivetrain.rotateToBigAbsoluteAngle(-135);

        drivetrain.stop();

        robot.markerExtend1.setPower(1);
        robot.markerExtend2.setPower(1);

        sleep(1500);

        robot.markerExtend1.setPower(0);
        robot.markerExtend2.setPower(0);
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