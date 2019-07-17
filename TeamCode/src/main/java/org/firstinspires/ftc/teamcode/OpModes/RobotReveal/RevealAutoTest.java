package org.firstinspires.ftc.teamcode.OpModes.RobotReveal;
import org.junit.Test;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelControl.PurePursuit.warriorlib.Path;

public class RevealAutoTest {

    private Path samplePath = new Path();
    private MaelPose startingPose = new MaelPose(0,0);
    private MaelPose goldPose = new MaelPose(3,3);

    private MaelPose middlePose = new MaelPose(0,1.5);
    private MaelPose wallPose = new MaelPose(-5,3);
    private MaelPose depotPose = new MaelPose(-10,3);

    private Path depot = new Path();


    @Test
    public void test(){

        samplePath.addPoints(startingPose,goldPose);
        depot.addPoints(startingPose,middlePose,wallPose,depotPose);


        Path path = samplePath;


        write("Initial Path: ");
        listpoints(path);
        write("Initial Path Listing complete");
        write("");

        write("Injected Path: ");
        path.setSpacing(.6);
        path.injectPoints();
        listpoints(path);
        write("Injected Path Listing complete");
        write("");

        write("Smooth Path: ");
        path.smooth(.01,0.001,0.001);
        listpoints(path);
        write("Smooth Path Listing complete");
        write("");

        write("Reversed Path: ");
        path.reverse();
        listpoints(path);
        write("Reversed Path Listing complete");
        write("");
    }

    private void write(String m){System.out.println(m);}

    private String pose(MaelPose pose){
        return  "(" + pose.x + ", " + pose.y + ")";
    }

    private void listpoints(Path p){
        for(int i = 0; i < p.numOfPoints(); i++){
            MaelPose pose = p.getPose(i);
            write("Point " + i + ": " + pose(pose));
        }
        write("End Point: " + pose(p.endPose()));

    }
}
