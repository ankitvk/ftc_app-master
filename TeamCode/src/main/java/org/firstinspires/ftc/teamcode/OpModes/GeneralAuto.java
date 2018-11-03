package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.PathFollower;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;


@Autonomous(name = "GeneralAuto")
public class GeneralAuto extends LinearOpMode implements AutonomousOpMode,Constants {

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
        goldfish.setAlignSettings(ALIGN_POSITION,1000);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.init(hardwareMap);
        goldfish.startOpenCV(hardwareMap); //start opencv

        waitForStart();

        robot.stopExtension.setPosition(0); //release robot

        extend();

        pivotForTime(-.75,500);

        retract();

        //drivetrain.rotateToAbsoluteAngle(-40);
        drivetrain.rotateForTime(0.75, 350);
        sleep(500);
        while(getOpModeIsActive() && !goldfish.getAligned()){
            drivetrain.rotate(0.45);
        }
        drivetrain.stop();
        goldfish.disable();

        drivetrain.driveForwardDistance(20);

       /* drivetrain.rotateToAbsoluteAngle(-robot.imu.getYaw());
        sleep(500);
        drivetrain.driveForwardDistance(30);

        *//*robot.intake.setPower(-.5);
        sleep(300);
        robot.intake.setPower(0);*//*

        drivetrain.rotateToAbsoluteAngle(-125);
        drivetrain.driveForwardDistance(84);*/
        drivetrain.stop();

    }//end opMode

    /*private void extend(){
        PIDController control = new PIDController(extensionKP,extensionKI,extensionKD,extensionMaxI);
        double error = Math.abs((robot.extensionLeft.getCurrentPosition()+(robot.extensionRight.getCurrentPosition()))/2-(NEVEREST20_COUNTS_PER_REV*5.5));
        double p;
        double target;
        double currentLoc;
        while(error>150){
            target = (NEVEREST20_COUNTS_PER_REV*4.5);
            currentLoc = robot.extensionLeft.getCurrentPosition();
            p = control.power(target,currentLoc);
            robot.extensionLeft.setPower(p);
            robot.extensionRight.setPower(p);
            error = Math.abs(target-currentLoc);
        }
    }
*/
    /*private void retract(){
        PIDController control = new PIDController(extensionKP,extensionKI,extensionKD,extensionMaxI);
        double error = Math.abs((robot.extensionLeft.getCurrentPosition()+(robot.extensionRight.getCurrentPosition()))/2);
        double p;
        double currentLoc;
        while(error>150){
            currentLoc = robot.extensionLeft.getCurrentPosition();
            p = control.power(NEVEREST20_COUNTS_PER_REV/2,currentLoc);
            robot.extensionLeft.setPower(p);
            robot.extensionRight.setPower(p);
            error = Math.abs(currentLoc);
        }
    }*/

    public void retract(){
        PIDController controlRetract = new PIDController(extensionKP,extensionKI,extensionKD,extensionMaxI);
        long startTime = System.nanoTime();
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 1000)){
            double position = robot.extensionLeft.getCurrentPosition();
            double power = controlRetract.power(NEVEREST20_COUNTS_PER_REV/2,position);

            robot.extensionLeft.setPower(power);
            robot.extensionRight.setPower(power);


            if (Math.abs(Math.abs(position)-Math.abs(NEVEREST20_COUNTS_PER_REV/2)) <= 150) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
        }
    }

    public void extend(){
        PIDController controlExtend = new PIDController(extensionKP,extensionKI,extensionKD,extensionMaxI);
        long startTime = System.nanoTime();
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 1000)){
            double position = robot.extensionLeft.getCurrentPosition();
            double power = controlExtend.power(NEVEREST20_COUNTS_PER_REV*4.5,position);

            robot.extensionLeft.setPower(power);
            robot.extensionRight.setPower(power);


            if (Math.abs(Math.abs(NEVEREST20_COUNTS_PER_REV*4.5)-Math.abs(position)) <= 150) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
        }
    }

    private void extendForTime(double power, double time){
        robot.extensionLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extensionLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.extensionRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extensionRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long startTime = System.nanoTime();
        long stopState = 0;
        while(stopState <= time){
            robot.extensionLeft.setPower(power);
            robot.extensionRight.setPower(power);
            stopState = (System.nanoTime() - startTime) / 1000000;
        }
        stop();
    }

    private void retractForTime(double power, double time){
        robot.extensionLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extensionLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.extensionRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extensionRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long startTime = System.nanoTime();
        long stopState = 0;
        while(stopState <= time){
            robot.extensionLeft.setPower(-power);
            robot.extensionRight.setPower(-power);
            stopState = (System.nanoTime() - startTime) / 1000000;
        }
        stop();
    }

    private void pivotForTime(double power, double time){
        robot.pivot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.pivot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long startTime = System.nanoTime();
        long stopState = 0;
        while(stopState <= time){
            robot.pivot1.setPower(power);
            robot.pivot2.setPower(power);
            stopState = (System.nanoTime() - startTime) / 1000000;
        }
        stop();
    }

    /*private void pivotDown(){
        PIDController control = new PIDController(.1,0,0,1)
            long startTime = System.nanoTime();
            long stopState = 0;
            while(opModeIsActive() && (stopState <= 1000)){
                double position = robot.pivot1.getCurrentPosition()
                double power = control.power(/*degrees,position);
                robot.pivot1.setPower(power);
                robot.pivot2.setPower(power);

                if (Math.abs(position-/*degrees) <= PIVOT_TOLERANCE) {
                    stopState = (System.nanoTime() - startTime) / 1000000;
                }
                else {
                    startTime = System.nanoTime();
                }
            }

        }
    }*/

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