package org.firstinspires.ftc.teamcode.OpModes.RobotReveal;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.teamcode.OpModes.Test.AutoBase;
import org.junit.Test;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelControl.PurePursuit.PathFollower;
import ftc.library.MaelControl.PurePursuit.warriorlib.Path;

public class StateMachineTest extends AutoBase {

    public enum progStates{
        addPoints,
        listInitialPath,
        injectPoints,
        listInjectedPath,
        smoothPath,
        listSmoothPath,
    }

    private boolean pointsListed = false;

    private Path samplePath = new Path();
    private MaelPose startingPose = new MaelPose(0,0);
    private MaelPose goldPose = new MaelPose(3,3);

    private MaelPose middlePose = new MaelPose(0,1.5);
    private MaelPose wallPose = new MaelPose(-5,3);
    private MaelPose depotPose = new MaelPose(-10,3);

    private Path depot = new Path();

    private PathFollower follower = new PathFollower();

    private  Path path = depot;

    @Test
    public void test(){
        follower.setPath(path);
        programStage = progStates.addPoints.ordinal();

        try{
        run();
        }
        catch (InterruptedException ex){
            ex.printStackTrace();
        }
/*
        samplePath.addPoints(startingPose,goldPose);
        depot.addPoints(startingPose,middlePose,wallPose,depotPose);


        //Path path = samplePath;


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
        write("");*/
    }

    private void write(String m){System.out.println(m);}

    private String pose(MaelPose pose){
        return  "(" + pose.x + ", " + pose.y + ")";
    }

    private void listpoints(Path p){
        pointsListed = false;
        for(int i = 0; i < p.numOfPoints(); i++){
            MaelPose pose = p.getPose(i);
            write("Point " + i + ": " + pose(pose));
        }
        write("End Point: " + pose(p.endPose()));
        pointsListed = true;

    }


    //addPoints,

    //listInitialPath,

    //injectPoints,

    //listInjectedPath,

    //smoothPath,

    //listSmoothPath

    @Override
    public int getOrdinalLength() {
        return progStates.values().length;
    }

    @Override
    public void MainStateMachine() {
        initializeStateVariables();

        if(programStage == progStates.addPoints.ordinal()){
            initializeStateVariables();
            samplePath.addPoints(startingPose,goldPose);
            depot.addPoints(startingPose,middlePose,wallPose,depotPose);
            nextStage();

        }

        if(programStage == progStates.listInitialPath.ordinal()){
            initializeStateVariables();
            write("Initial Path: ");
            listpoints(path);
            write("Initial Path Listing complete");
            write("");

            if(pointsListed) nextStage();
        }

        if(programStage == progStates.injectPoints.ordinal()){
            initializeStateVariables();
            path.setSpacing(.6);
            path.injectPoints();
            nextStage();
        }

        if(programStage == progStates.listInjectedPath.ordinal()){
            initializeStateVariables();
            write("Injected Path: ");
            listpoints(path);
            write("Injected Path Listing complete");
            write("");

            if(pointsListed) nextStage();
        }

        if(programStage == progStates.smoothPath.ordinal()){
            initializeStateVariables();
            path.smooth(.01,0.001,0.001);
            nextStage();
        }

        if(programStage == progStates.listSmoothPath.ordinal()){
            initializeStateVariables();
            write("Smooth Path: ");
            listpoints(path);
            write("Smooth Path Listing complete");
            write("");

            if(pointsListed) nextStage();
        }

    }
}

