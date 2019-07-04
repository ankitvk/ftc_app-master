package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Leviathan;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelWrappers.MaelLinearOp;

@TeleOp(name = "TankOdometryTest")
public class TankOdometryTest extends MaelLinearOp implements Constants {

    private Leviathan leviathan = new Leviathan();
    
    @Override
    public void run() throws InterruptedException {

/*        leviathan.initHardware(hardwareMap);
        leviathan.tankTracker.gearRatio = DT_GEAR_RATIO;
        MaelPose currPoint = leviathan.tankTracker.toPose();

        while(!opModeIsActive()){
            feed.add("Tank Odometry Test");
            feed.add("Op Mode State Active? : ", opModeIsActive());
            feed.add("Tracker X: ",currPoint.x);
            feed.add("Tracker Y: ", currPoint.y);
            feed.add("Distance: ", leviathan.tankTracker.trackDistance());
            feed.add("Angle: ", currPoint.angle);
            feed.update();
        }

        waitForStart();

        while(opModeIsActive()){
            leviathan.driveTeleop(controller1);
            feed.add("Op Mode State Active? : ", opModeIsActive());
            feed.add("Tracker X: ",currPoint.x);
            feed.add("Tracker Y: ", currPoint.y);
            feed.add("Distance: ", leviathan.tankTracker.trackDistance());
            feed.add("Angle: ", currPoint.angle);
            feed.update();
        }*/
    }

    @Override
    public void initHardware() {

    }
}
