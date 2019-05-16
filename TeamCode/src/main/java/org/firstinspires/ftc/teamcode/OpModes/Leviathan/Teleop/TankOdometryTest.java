package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Leviathan;

import ftc.library.MaelWrappers.MaelLinearOp;

@TeleOp(name = "TankOdometryTest")
public class TankOdometryTest extends MaelLinearOp {

    private Leviathan leviathan = new Leviathan();
    
    @Override
    public void run() throws InterruptedException {

        leviathan.initHardware(hardwareMap);

        while(!opModeIsActive()){
            feed.add("Tank Odometry Test");
            feed.add("Op Mode State Active? : ", opModeIsActive());
            feed.add("Tracker X: ",leviathan.tracker.trackX());
            feed.add("Tracker Y: ", leviathan.tracker.trackY());
            feed.add("Distance: ", leviathan.tracker.trackDistance());
            feed.add("Angle: ", leviathan.tracker.trackAngle());
            feed.update();
        }

        waitForStart();

        while(opModeIsActive()){
            leviathan.driveTeleop(controller1);

            feed.add("Op Mode State Active? : ", opModeIsActive());
            feed.add("Tracker X: ",leviathan.tracker.trackX());
            feed.add("Tracker Y: ", leviathan.tracker.trackY());
            feed.add("Distance: ", leviathan.tracker.trackDistance());
            feed.add("Angle: ", leviathan.tracker.trackAngle());
            feed.update();
        }
    }
}
