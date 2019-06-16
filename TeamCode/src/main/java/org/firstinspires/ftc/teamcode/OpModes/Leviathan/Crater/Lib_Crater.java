package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Crater;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Leviathan;

import ftc.library.MaelSensors.MaelTimer;
import ftc.library.MaelUtils.MaelUtils;
import ftc.library.MaelWrappers.MaelLinearOp;

@Autonomous(name = "Tarun's CraterAuto", group = "Lib_Auto")
public class Lib_Crater extends MaelLinearOp implements Constants {

    public Leviathan robot = new Leviathan();
    public MaelTimer timer = new MaelTimer();
    String blockPos;

    @Override
    public void run() throws InterruptedException {
        //robot.setAutoOpMode(this);
        //robot.initHardware(hardwareMap);

        //robot.index.setPos(.15);

/*        robot.hangLeftRealease.setPos(.3);
        robot.hangRightRelease.setPos(.85);

        while(!opModeIsActive()){
            if(robot.getGoldPos() == GoldPos.LEFT) blockPos = "LEFT";
            else if(robot.getGoldPos() == GoldPos.MIDDLE) blockPos = "MIDDLE";
            else if(robot.getGoldPos() == GoldPos.RIGHT) blockPos = "RIGHT";
            feed.add("Limit:",robot.limit.getSignalValue());
            feed.add("Pivot Angle:",robot.pivot.getAngle());
            feed.add("Gold Position:",blockPos);
            feed.update();
        }


        Runnable pivot = new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive() && !timer.elapsedTime(3, MaelTimer.Time.SECS)){
                    robot.pivot.scoringPosition();
                }
            }
        };

        Runnable extend = new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive())
                    robot.lift.runToDistance(20,0.8);
            }
        };

        Runnable lower = new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive() && !timer.elapsedTime(2, MaelTimer.Time.SECS)){
                    robot.lift.setPower(.75);
                }
            }
        };

        Runnable drive5 = new Runnable() {
            @Override
            public void run() {
                robot.driveDistance(5);
            }
        };*/
        while(!opModeIsActive()) {
            timer.reset();
            feed.add("Time:", timer.secs());
            feed.update();
        }

        //PIDController yeah = new PIDController(1,1,1,1);
        //double y = yeah.power(20,10);

        waitForStart();

        long startTime = timer.startTime();
        feed.add("starttime:",startTime);
        feed.update();
        if(startTime > 10000000) {
            long stopState = timer.stopState();
        }

        feed.add("Start time:",startTime);
        feed.add("Stop state:",timer.stopState());
        feed.update();

        //feed.add("Error:",y);
        //feed.update();
        //sleep(10);

    }

}
