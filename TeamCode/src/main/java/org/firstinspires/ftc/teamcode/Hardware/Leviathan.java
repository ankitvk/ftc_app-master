package org.firstinspires.ftc.teamcode.Hardware;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.GoldPos;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Subsystems.Components.AutoMation;
import org.firstinspires.ftc.teamcode.Subsystems.Components.MaelPivot;

import ftc.library.MaelControl.PID.PIDFController;
import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelControl.PurePursuit.PathFollower;
import ftc.library.MaelMotions.MaelMotors.Motor;
import ftc.library.MaelMotions.MaelServos.CRServo.MaelCRServoSystem;
import ftc.library.MaelMotions.MaelServos.Servo.MaelServo;
import ftc.library.MaelRobot;
import ftc.library.MaelSensors.MaelIMU;
import ftc.library.MaelSensors.MaelLimitSwitch;
import ftc.library.MaelSensors.MaelOdometry.TankOdometry;
import ftc.library.MaelSensors.MaelTimer;
import ftc.library.MaelSubsystems.MaelCollector;
import ftc.library.MaelSubsystems.MaelstromDrivetrain.DrivetrainModels;
import ftc.library.MaelSubsystems.MaelstromDrivetrain.MaelDrivetrain;
import ftc.library.MaelSubsystems.MaelElevator;
import ftc.library.MaelUtils.SubsystemModels;
import ftc.library.MaelWrappers.MaelController;
import ftc.library.MaelWrappers.MaelTellemetry;

//taruns stuff
public class Leviathan extends MaelRobot implements Constants {

    public MaelCollector intake;
    public MaelElevator lift;
    public MaelPivot pivot;
    public MaelCRServoSystem hang;
    public MaelServo index;
    public MaelServo hangLeftRealease, hangRightRelease;
    public AutoMation automation;
    public MaelLimitSwitch limit;
    public Dogeforia dogeForia;
    public GoldAlignDetector detector;
    public PathFollower follower;
    GoldPos position;
    public boolean startOpenCV;
    private PIDFController liftPid = new PIDFController(0.01,0,0,0,1);
    private PIDFController pivtorPid = new PIDFController(0.01,0,0,0,0);

    @Override
    public void initHardware(HardwareMap hwMap) {
        dt = new MaelDrivetrain(DrivetrainModels.ARCADE,DT_GEAR_RATIO,dtKP,dtKI,dtKD,hwMap, Motor.NEVEREST_NAKED,this);
        setSpeedMultiplier(.85);
        imu = new MaelIMU("imu",hwMap);
        feed = MaelTellemetry.getFeed();
        tankTracker = new TankOdometry(dt,imu);
        intake = new MaelCollector("winch", Motor.ORBITAL20, SubsystemModels.MOTOR,hwMap);
        intake.setCollectorPowers(INTAKE_POWER,OUTTAKE_POWER);
        lift = new MaelElevator("extendo", Motor.NEVEREST_NAKED,SubsystemModels.MOTOR,hwMap);
        lift.setLiftPowers(LIFT_EXTEND,LIFT_RETRACT);
        lift.setGearboxRatio(14);
        lift.setSpoolDiameter(2);
        limit = new MaelLimitSwitch("limit",hwMap);
        pivot = new MaelPivot(hwMap);
        pivot.setPivotPowers(PIVOT_UP,PIVOT_DOWN);
        pivot.setLimit(limit);
        pivot.setFeed(feed);
        hang = new MaelCRServoSystem("hangLeftBottom","hangLeftTop","hangRightBottom","hangRightTop",hwMap);
        hang.setDirection(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
        index = new MaelServo("index",hwMap);
        hangLeftRealease = new MaelServo("hangLeftRelease",hwMap);
        hangRightRelease = new MaelServo("hangRightRelease",hwMap);
        if(startOpenCV) startOpenCV(hwMap);
        follower = new PathFollower(this);
        automation = new AutoMation(this);
        add(intake,lift,pivot,imu);
        setPidConstants();
        follower.setLookAhead(.5);
        follower.setTrackWidth(LENGTH_BETWEEN_WHEELS);
    }

    public void setPidConstants(){
        distancePid.setPID(distanceKP,distanceKI,distanceKD);
        turnPid.setPID(turnKP,turnKI,turnKD);
        sideTurnPid.setPID(sideKP,sideKI,sideKD);
        follower.setEndKp(DRIVE_TO_DEPOT_KP);
    }

    public void hangRelease(){
        hangLeftRealease.setPos(.75);
        hangRightRelease.setPos(.25);
    }

    public void setStartOpenCV(boolean bool){
        this.startOpenCV = bool;
    }

    public void drop(){
        MaelTimer hangTimer = new MaelTimer();
        hang.setPower(.85);
        hangRelease();
        hangTimer.startTime();
        while(isStopRequested() && (hangTimer.stopState() <= 2500)){
            hang.setPower(-.85);
        }
        hang.setPower(0);
    }


    public void extendDistance(double distance){

        long startTime = System.nanoTime();
        long stopState = 0;

        while((isStopRequested() && (stopState <= 250))){
            double currDistance = lift.getDistance();
            double power = liftPid.power(distance,currDistance);

            lift.setPower(power);

            feed.add("Current Distance: ", currDistance);
            feed.add("Power: ", power);
            feed.add("Error: ", liftPid.getError());
            feed.add("P: ",liftPid.getP());
            feed.add("I: ",liftPid.getI());
            feed.add("D: ",liftPid.getD());
            feed.update();

            if(liftPid.getError() <= .5) stopState = (System.nanoTime() - startTime) / 1000000;
            else startTime = System.nanoTime();
        }
        stop();
    }

    public void startOpenCV (HardwareMap hwMap) {

        detector = new GoldAlignDetector();
        detector.init(hwMap.appContext, CameraViewDisplay.getInstance(),0,true);
        detector.useDefaults();
        detector.alignSize = 200;
        detector.alignPosOffset = 0;
        detector.downscale = .4;
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.COLOR_DEVIATION;
        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = LICENSE_KEY;
        parameters.fillCameraMonitorViewParent = true;
        parameters.cameraName = hwMap.get(WebcamName.class,"Webcam 1");
        dogeForia = new Dogeforia(parameters);
        dogeForia.enableConvertFrameToBitmap();
        dogeForia.setDogeCVDetector(detector);
        dogeForia.enableDogeCV();
        dogeForia.showDebug();
        dogeForia.start();
    }

    public void startOpenCVPhone(HardwareMap hwMap){
        detector = new GoldAlignDetector();
        detector.init(hwMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.downscale = 0.4; // How much to downscale the input frames
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //falcon.goldAlignDetector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable();
    }

    public GoldPos getGoldPos(){
        double block = detector.getXPosition();
        if(!detector.isFound()) return position = GoldPos.RIGHT;
        else if(block > 340 ) return position = GoldPos.MIDDLE;
        else if(block <= 340) return position = GoldPos.LEFT;
        return position;
    }

    public MaelPose goldPose(){
        if(position == GoldPos.LEFT) return new MaelPose(-3,3);
        else if(position == GoldPos.MIDDLE) return new MaelPose(0,3);
        else if(position == GoldPos.RIGHT) return new MaelPose(3,3);
        else return new MaelPose(0,3);
    }

    public void index(MaelController controller){
        if(controller.a()) index.setPos(.15);
        else index.setPos(.95);
    }

    public void intakeTeleop(MaelController c){
        if(c.leftBumper()){
            intake.setState(MaelCollector.State.INTAKE);
        }
        else if(c.leftTriggerPressed()){
            intake.setState(MaelCollector.State.OUTTAKE);
        }
        else intake.setState(MaelCollector.State.STOP);

        /*if(c.a()) index.setPos(.15);
        else index.setPos(.95);*/
    }

    public void pivotTeleop(MaelController c){
        if(c.leftBumper()) pivot.setState(MaelPivot.State.UP);
        else if(c.rightBumper()) pivot.setState(MaelPivot.State.DOWN);
        else pivot.setState(MaelPivot.State.STOP);
    }

    public void liftTeleop(MaelController c){
        if(c.rightTriggerPressed()) lift.setState(MaelElevator.State.EXTEND);
        else if(c.rightBumper()) lift.setState(MaelElevator.State.RETRACT);
        else lift.setState(MaelElevator.State.STOP);
    }

    public void hangTeleop(MaelController c){
        if(c.leftTriggerPressed()) lift();
        else if(c.rightTriggerPressed()) lower();
        else hang.setPower(0);
    }

    public void teleop(MaelController controller1, MaelController controller2){
        driveTeleop(controller1);
        intakeTeleop(controller1);
        index(controller2);
        pivotTeleop(controller2);
        liftTeleop(controller1);
        hangTeleop(controller2);
    }

    public void lift(){
        hang.setPower(-.85);
    }

    public void lower(){
        hang.setPower(.85);
    }


    public void stopAllSystems(){
        stop();
        intake.setState(MaelCollector.State.STOP);
        pivot.setState(MaelPivot.State.STOP);
        lift.setState(MaelElevator.State.STOP);
        index.setPos(.95);
        hang.setPower(0);
    }
}
