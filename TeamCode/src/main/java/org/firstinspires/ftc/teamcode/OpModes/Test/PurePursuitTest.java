package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Opportunity;
import org.junit.Test;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelControl.PurePursuit.PathFollower;
import ftc.library.MaelControl.PurePursuit.warriorlib.Path;
import ftc.library.MaelWrappers.MaelLinearOp;

@Autonomous(name = "Pure Pursuit Test",group = "PurePursuit")
public class PurePursuitTest extends MaelLinearOp {
    private Opportunity robot =  new Opportunity();
    private PathFollower purePursuit;
    private MaelPose point0 = new MaelPose(0,0);
    private MaelPose point1 = new MaelPose(3,3);
    private MaelPose point2 = new MaelPose(4,5);
    private Path path = new Path();

    @Override
    public void run() throws InterruptedException {
        while(!opModeIsActive()){
            robot.tankTracker.update();
            feed.add("X: ",robot.tankTracker.getX());
            feed.add("Y: ",robot.tankTracker.getY());
            feed.add("Heading: ",robot.tankTracker.getHeading());
            feed.update();
        }
        waitForStart();
        robot.stop();
        sleep(3);
/*        while(!isStopRequested()){
            robot.tankTracker.update();
            purePursuit.followPath(.4);
            feed.add("X: ",robot.tankTracker.getX());
            feed.add("Y: ",robot.tankTracker.getY());
            feed.add("Heading: ",robot.tankTracker.getHeading());
            feed.add("LookAhead X: ",purePursuit.getLookAheadPoint().x);
            feed.add("LookAhead Y: ",purePursuit.getLookAheadPoint().y);
            feed.add("Left Power ",robot.dt.leftDrive.getPower());
            feed.add("Right Power ",robot.dt.rightDrive.getPower());
            feed.update();
            if(purePursuit.distanceError <= 5) break;
            if(isStopRequested()) break;
        }
        robot.stop();*/
        purePursuit.followPath(0.6);
    }

    /*@Test
    public void test(){
        ArrayList<MaelPose> points = new ArrayList<>();
        Path path = new Path();

        path.addPoints(new MaelPose(3,4),new MaelPose(5,8));

        purePursuit = new PathFollower(robot);
        //add(point0,point1,point2);
        //points.add(point1);
        //purePursuit.wayPoints = points;
        purePursuit.setLookAhead(3);
        purePursuit.distanceBetweenWheels = 16;
        //path.addPoints(point0,point1,point2);
        purePursuit.setPath(path);

        purePursuit.followPath(.5);
    }
*/



    @Override
    public void initHardware() {
        robot.initHardware(hardwareMap);
        purePursuit = new PathFollower(robot);
        //add(point0,point1,point2);
        //points.add(point1);
        //purePursuit.wayPoints = points;
        purePursuit.setLookAhead(3);
        purePursuit.distanceBetweenWheels = 16;
        path.addPoints(point0,point1,point2);
        purePursuit.setPath(path);
    }
}
