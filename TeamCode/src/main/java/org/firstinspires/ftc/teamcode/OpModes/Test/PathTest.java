package org.firstinspires.ftc.teamcode.OpModes.Test;
import org.junit.Test;

import java.util.Scanner;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelControl.PurePursuit.warriorlib.Path;

public class PathTest {
    Scanner sc = new Scanner(System.in);
    int timeToStep = 0;
    public static int currentTime = 0;
    MaelPose point1 = new MaelPose(0,0);
    MaelPose point2 = new MaelPose(0,5);
    MaelPose point3 = new MaelPose(5,5);
    MaelPose point4 = new MaelPose(5,10);

    private void write(String m){System.out.println(m);}

    private String pose(MaelPose pose){
        return  "(" + pose.x + ", " + pose.y + ")";
    }
    @Test
    public void test(){
        Path path = new Path();
        path.addPoints(point1,point2,point3,point4);
        path.setSpacing(.7);
        int timeToStep = 0;
        write("Initial Path: ");
        for(int i = 0; i < path.numOfPoints(); i++){
            MaelPose pose = path.getPose(i);
            //write("Point " + i + ": (" + pose.x + "," + pose.y + ")");
            write("Point " + i + ": " + pose(pose));
        }
        write("End Point: " + pose(path.endPose()));
        write("Initial Path Listing complete");
        write("");

        path.injectPoints();

        write("Injected Path: ");
        for(int i = 0; i < path.numOfPoints() - 1; i++){
            MaelPose pose = path.getPose(i);
            //write("Point " + i + ": (" + pose.x + "," + pose.y + ")");
            write("Point " + i + ": " + pose(pose));
        }
        write("End Point: " + pose(path.endPose()));
        write("Injected Path Listing complete");
        write("");

        /*Path smooth = path.smooth(.2,.8,0.001);

        write("Smooth Path: ");
        for(int i = 0; i < smooth.numOfPoints() - 1; i++){
            MaelPose pose = smooth.getPose(i);
            //write("Point " + i + ": (" + pose.x + "," + pose.y + ")");
            write("Point " + i + ": " + pose(pose));
        }
        write("End Point: " + pose(smooth.endPose()));
        write("Smooth Path Listing complete");
        write("");*/

        path.smoothPath(.2,.8,0.001);

        write("Smooth Path: ");
        for(int i = 0; i < path.numOfPoints() - 1; i++){
            MaelPose pose = path.getPose(i);
            //write("Point " + i + ": (" + pose.x + "," + pose.y + ")");
            write("Point " + i + ": " + pose(pose));
        }
        write("End Point: " + pose(path.endPose()));
        write("Smooth Path Listing complete");
        write("");

        Path reversed = path.reverse();

        write("Reversed Path: ");
        for(int i = 0; i < reversed.numOfPoints() - 1; i++){
            MaelPose pose = reversed.getPose(i);
            //write("Point " + i + ": (" + pose.x + "," + pose.y + ")");
            write("Point " + i + ": " + pose(pose));
        }
        write("End Point: " + pose(reversed.endPose()));
        write("Reversed Path Listing complete");
        write("");

    }


}
