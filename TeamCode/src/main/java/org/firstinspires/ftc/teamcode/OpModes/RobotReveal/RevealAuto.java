package org.firstinspires.ftc.teamcode.OpModes.RobotReveal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.GoldPos;
import org.firstinspires.ftc.teamcode.Hardware.Leviathan;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelControl.PurePursuit.warriorlib.Path;
import ftc.library.MaelSubsystems.MaelCollector;
import ftc.library.MaelSubsystems.MaelElevator;
import ftc.library.MaelWrappers.MaelLinearOp;

@Autonomous(name = "Reveal Auto")
public class RevealAuto extends MaelLinearOp implements Constants {
    private Leviathan robot = new Leviathan();
    private String side = "CRATER";

    private MaelPose startingPose = new MaelPose(0,0);
    private MaelPose middlePose = new MaelPose(0,1.5);
    private MaelPose wallPose = new MaelPose(-5,3);
    private MaelPose depotPose = new MaelPose(-10,3);
    private MaelPose goldPose = new MaelPose();

    private Path depot = new Path();
    private Path samplePath = new Path();

    private GoldPos goldPos = GoldPos.UNKNOWN;

    @Override
    public void run() throws InterruptedException {
        while(isStopRequested()){
            robot.tankTracker.update();
            goldPos = robot.getGoldPos();
            goldPose = robot.goldPose();

            if(controller1.aToggle()){
                side = "DEPOT";
            }
            else side = "CRATER";

            feed.add("X :",robot.tankTracker.getX());
            feed.add("Y :",robot.tankTracker.getX());
            feed.add("Imu :",robot.imu.getYaw());
            feed.add("Current Gold Position: ", goldPos);
            feed.add("Side: ", side);
            feed.update();
        }

        waitForStart();

        robot.drop();

        if(side.equals("CRATER"))craterAuto();
        else if(side.equals("DEPOT")) depotAuto();

        //extend and park in crater
        robot.lift.setState(MaelElevator.State.EXTEND);
        sleep(2.5);
        robot.lift.setState(MaelElevator.State.STOP);



    }

    private void craterAuto(){
        //sample
        robot.follower.followPath(.7);

        samplePath.reverse();

        //go back to initial pose
        robot.follower.followPath(.5);

        robot.follower.setPath(depot);

        robot.follower.followPath(.7);

        //drop marker
        robot.intake.setState(MaelCollector.State.OUTTAKE);
        sleep(2.5);
        robot.intake.setState(MaelCollector.State.STOP);


        depot.reverse();

        //return to initial pose
        robot.follower.followPath(.5);


    }

    private void depotAuto(){
        //sample
        robot.follower.followPath(.7);

        samplePath.reverse();

        //go back to initial pose
        robot.follower.followPath(.5);

        //extend towards depot
        robot.lift.setState(MaelElevator.State.EXTEND);
        sleep(2.5);
        robot.lift.setState(MaelElevator.State.STOP);

        //drop marker
        robot.intake.setState(MaelCollector.State.OUTTAKE);
        sleep(2.5);
        robot.intake.setState(MaelCollector.State.STOP);

        //retract
        robot.lift.setState(MaelElevator.State.RETRACT);
        sleep(2.5);
        robot.lift.setState(MaelElevator.State.STOP);

        depot.clear();
        depot.addPoints(startingPose,middlePose,wallPose);

        robot.follower.setPath(depot);

        //go to back crater
        robot.follower.followPath(.7);

    }



    @Override
    public void initHardware() {
        robot.setStartOpenCV(true);
        robot.initHardware(hardwareMap);
        //robot.startOpenCV(hardwareMap);
        samplePath.addPoints(startingPose,goldPose);
        samplePath.setSpacing(.7);
        samplePath.injectPoints();
        robot.follower.setPath(samplePath);

        depot.addPoints(startingPose,middlePose,wallPose,depotPose);
        depot.setSpacing(.7);
        depot.injectPoints();
    }
}
