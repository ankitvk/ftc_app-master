package org.firstinspires.ftc.teamcode.OpModes.PurePursuit;

import org.firstinspires.ftc.teamcode.Hardware.Opportunity;
import org.junit.Test;

import java.util.ArrayList;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelControl.PurePursuit.PathFollower;
import ftc.library.MaelControl.PurePursuit.warriorlib.Path;

public class PurePursuitControllerTest {
    private Opportunity robot =  new Opportunity();
    private PathFollower purePursuit;
    private MaelPose point0 = new MaelPose(0,0);
    private MaelPose point1 = new MaelPose(-4,3);
    private MaelPose point2 = new MaelPose(-10,5);
    private Path path = new Path();
    @Test
    public void test(){
        ArrayList<MaelPose> points = new ArrayList<>();
        Path path = new Path();

        path.addPoints(new MaelPose(3,4),new MaelPose(5,8));

        purePursuit = new PathFollower(robot);
        //add(point0,point1,point2);
        //points.add(point1);
        //purePursuit.wayPoints = points;
        purePursuit.setLookAhead(3);
        purePursuit.setTrackWidth(16);
        path.addPoints(point0,point1,point2);
        purePursuit.setPath(path);
    }
}
