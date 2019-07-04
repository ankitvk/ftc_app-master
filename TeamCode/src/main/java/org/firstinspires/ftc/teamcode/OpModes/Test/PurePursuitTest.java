package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Opportunity;

import java.util.ArrayList;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelControl.PurePursuit.PathFollower;
import ftc.library.MaelWrappers.MaelLinearOp;

@Autonomous(name = "Pure Pursuit Test",group = "PurePursuit")
public class PurePursuitTest extends MaelLinearOp {
    private Opportunity robot =  new Opportunity();
    private PathFollower purePursuit;
    private ArrayList<MaelPose> path = new ArrayList<>();
    MaelPose point1 = new MaelPose(10,10);
    @Override
    public void run() throws InterruptedException {


        while(!opModeIsActive()){
            feed.add("X: ",robot.tankTracker.getX());
            feed.add("Y: ",robot.tankTracker.getY());
            feed.add("Heading: ",robot.tankTracker.getHeading());
            feed.update();
        }

        waitForStart();

        robot.stop();
        sleep(3);

        while(!isStopRequested()){
            purePursuit.followPath(85);

            feed.add("X: ",robot.tankTracker.getX());
            feed.add("Y: ",robot.tankTracker.getY());
            feed.add("Heading: ",robot.tankTracker.getHeading());
            feed.add("LookAhead X: ",purePursuit.getLookAheadPoint().x);
            feed.add("LookAhead Y: ",purePursuit.getLookAheadPoint().y);
            feed.add("Left Power ",robot.dt.leftDrive.getPower());
            feed.add("Right Power ",robot.dt.rightDrive.getPower());
            feed.update();

            if(isStopRequested()) break;
        }

        robot.stop();



    }

    @Override
    public void initHardware() {
        robot.initHardware(hardwareMap);
        purePursuit = new PathFollower(robot);
        path.add(point1);
        purePursuit.wayPoints = path;
        purePursuit.setLookAhead(10);
        purePursuit.distanceBetweenWheels = 16;
        robot.dt.eReset();
    }
}
