package org.firstinspires.ftc.teamcode.OpModes.RobotReveal;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Leviathan;
import org.firstinspires.ftc.teamcode.OpModes.Test.AutoBase;

import ftc.library.MaelWrappers.MaelLinearOp;

@TeleOp(name = "Reveal Teleop")
public class RevealTeleop extends MaelLinearOp implements Constants {

    private Leviathan robot = new Leviathan();

    @Override
    public void run() throws InterruptedException {

        while(isStopRequested()){
            feed.add("Heading: ", robot.imu.getYaw());
            feed.add("Pivot Angle: ", robot.pivot.getAngle());
            feed.add("Lift Distance: ", robot.lift.getDistance());
            feed.update();
        }

        waitForStart();

        while(!isStopRequested()){
            robot.teleop(controller1,controller2);

            if(controller1.x()) robot.automation.loopStateMachine();

            feed.add("X: :", robot.tankTracker.getX());
            feed.add("Y: ", robot.tankTracker.getY());
            feed.add("Heading: ", robot.imu.getYaw());
            feed.add("Left Power:", robot.dt.leftDrive.getPower());
            feed.add("Right Power:", robot.dt.rightDrive.getPower());
            feed.add("Pivot Angle:",robot.pivot.getAngle());
            feed.add("Lift Distance:",robot.lift.getDistance());
            feed.update();

        }
    }

    @Override
    public void initHardware() {
        robot.initHardware(hardwareMap);
    }
}
