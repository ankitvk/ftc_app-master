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
import org.firstinspires.ftc.teamcode.Subsystems.Components.MaelstromPivot;

import ftc.library.MaelstromMotions.MaelstromMotors.Motor;
import ftc.library.MaelstromMotions.MaelstromServos.CRServo.MaelstromCRServoSystem;
import ftc.library.MaelstromMotions.MaelstromServos.Servo.MaelstromServo;
import ftc.library.MaelstromRobot;
import ftc.library.MaelstromSensors.MaelstromIMU;
import ftc.library.MaelstromSensors.MaelstromLimitSwitch;
import ftc.library.MaelstromSensors.MaelstromTimer;
import ftc.library.MaelstromSubsystems.MaelCollector;
import ftc.library.MaelstromSubsystems.MaelstromDrivetrain.DrivetrainModels;
import ftc.library.MaelstromSubsystems.MaelstromDrivetrain.MaelstromDrivetrain;
import ftc.library.MaelstromSubsystems.MaelstromElevator;
import ftc.library.MaelstromUtils.MaelstromUtils;
import ftc.library.MaelstromUtils.SubsystemModels;
import ftc.library.MaelstromWrappers.MaelstromController;
import ftc.library.MaelstromWrappers.MaelstromTellemetry;

//taruns stuff
public class Leviathan extends MaelstromRobot implements Constants {

    public MaelCollector intake;
    public MaelstromElevator lift;
    public MaelstromPivot pivot;
    public MaelstromCRServoSystem hang;
    public MaelstromServo index;
    public MaelstromServo hangLeftRealease, hangRightRelease;
    public MaelstromLimitSwitch limit;
    public Dogeforia dogeForia;
    public GoldAlignDetector detector;
    GoldPos position;
    public boolean startOpenCV;

    @Override
    public void initHardware(HardwareMap hwMap) {
        dt = new MaelstromDrivetrain(DrivetrainModels.ARCADE,DT_GEAR_RATIO,dtKP,dtKI,dtKD,hwMap, Motor.NEVEREST_NAKED,this);
        setSpeedMultiplier(.5);
        imu = new MaelstromIMU("imu",hwMap);
        feed = MaelstromTellemetry.getFeed();
        intake = new MaelCollector("winch", Motor.ORBITAL20, SubsystemModels.MOTOR,hwMap);
        intake.setCollectorPowers(INTAKE_POWER,OUTTAKE_POWER);
        lift = new MaelstromElevator("extendo", Motor.NEVEREST_NAKED,SubsystemModels.MOTOR,hwMap);
        lift.setLiftPowers(LIFT_EXTEND,LIFT_RETRACT);
        limit = new MaelstromLimitSwitch("limit",hwMap);
        pivot = new MaelstromPivot(hwMap);
        pivot.setPivotPowers(PIVOT_UP,PIVOT_DOWN);
        pivot.setLimit(limit);
        pivot.setFeed(feed);
        hang = new MaelstromCRServoSystem("hangLeftBottom","hangLeftTop","hangRightBottom","hangRightTop",hwMap);
        hang.setDirection(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
        index = new MaelstromServo("index",hwMap);
        hangLeftRealease = new MaelstromServo("hangLeftRelease",hwMap);
        hangRightRelease = new MaelstromServo("hangRightRelease",hwMap);
        if(startOpenCV) startOpenCV(hwMap);
        //hangRealease = new MaelstromServoSystem("hangLeftRelease","hangRightRelease",hwMap);
    }

    public void setPidConstants(){
        distancePid.setPID(distanceKP,distanceKI,distanceKD);
        turnPid.setPID(turnKP,turnKI,turnKD);
        sideTurnPid.setPID(sideKP,sideKI,sideKD);
    }

    public void hangRelease(){
        hangLeftRealease.setPos(.75);
        hangRightRelease.setPos(.25);
    }

    public void setStartOpenCV(boolean bool){
        this.startOpenCV = bool;
    }

    public void drop(){
        MaelstromTimer hangTimer = new MaelstromTimer();
        hang.setPower(.85);
        hangRelease();
        hangTimer.startTime();
        while(opModeActive() && (hangTimer.stopState() <= 2500)){
            hang.setPower(-.85);
        }
        hang.setPower(0);
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

    public void index(MaelstromController controller){
        if(controller.a()) index.setPos(.15);
        else index.setPos(.95);
    }

    public void lift(){
        hang.setPower(-.85);
    }

    public void lower(){
        hang.setPower(.85);
    }

    public void setAutoOpMode(MaelstromUtils.AutonomousOpMode auto){this.auto = auto;}
}
