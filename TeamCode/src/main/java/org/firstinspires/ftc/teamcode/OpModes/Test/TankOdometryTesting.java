package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Opportunity;

import ftc.library.MaelSensors.MaelTimer;
import ftc.library.MaelUtils.TimeUnits;
import ftc.library.MaelWrappers.MaelLinearOp;

@Autonomous(name = "Tank Odometry Testing with Opportunity",group = "Odometry")
public class TankOdometryTesting extends MaelLinearOp implements Constants {
    private Opportunity robot = new Opportunity();

    @Override
    public void run() throws InterruptedException {
        robot.initHardware(hardwareMap);
        //hardwareThread.run();
        robot.dt.bl.setClosedLoop(true);
        robot.dt.bl.setPID(1,0,0);
        while(!opModeIsActive()){
            robot.tankTracker.update();
            feed.add("X: ",robot.tankTracker.getX());
            feed.add("Y: ",robot.tankTracker.getY());
            feed.add("Heading: ",robot.tankTracker.getHeading());
            feed.add("Imu: ",robot.imu.getYaw());
            feed.add("Velocity: ",robot.dt.bl.getVelocity());
            feed.update();
        }

        waitForStart();

/*        robot.drive(.5);
        sleep(2.5);
        robot.stop();*/

        drive(0.5,0.5,2);
        drive(-0.5,0.5,2);
        drive(0.3,0.3,2);
    }

    @Override
    public void initHardware() {

    }

    public void drive(double left, double right, double timeout){
        long startTime = System.nanoTime();
        long stopState = 0;
        while(opModeIsActive() && stopState <= (timeout * 1000)){
            robot.dt.bl.setVelocity(left);
            //robot.drive(left,right);
            robot.tankTracker.update();

            feed.add("Left: ",left);
            feed.add("Right: ",right);
            feed.add("X: ",robot.tankTracker.getX());
            feed.add("Y: ",robot.tankTracker.getY());
            feed.add("Heading: ",robot.tankTracker.getHeading());
            feed.add("Imu: ",robot.imu.getYaw());
            feed.add("Power: ",robot.dt.bl.getPower());
            feed.add("Velo Pid Power: ",robot.dt.bl.getPidPower());
            feed.add("Velocity: ",robot.dt.bl.getVelocity());
            feed.add("Target Vel: ",robot.dt.bl.getTargetVelocity(left));
            feed.add("Stop State: ",(stopState) / 1000);
            feed.update();

            stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
        }
        robot.stop();
    }
}
