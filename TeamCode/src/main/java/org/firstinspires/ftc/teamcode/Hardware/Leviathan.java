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
import org.firstinspires.ftc.teamcode.Subsystems.Components.MaelPivot;

import java.util.ArrayList;

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
import ftc.library.MaelSubsystems.Subsystem;
import ftc.library.MaelUtils.MaelUtils;
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
    public MaelLimitSwitch limit;
    public Dogeforia dogeForia;
    public GoldAlignDetector detector;
    GoldPos position;
    ArrayList<Subsystem> s;
    public boolean startOpenCV;

    @Override
    public void initHardware(HardwareMap hwMap) {
        dt = new MaelDrivetrain(DrivetrainModels.ARCADE, Companion.getDT_GEAR_RATIO(), Companion.getDtKP(), Companion.getDtKI(), Companion.getDtKD(),hwMap, Motor.NEVEREST_NAKED,this);
        setSpeedMultiplier(.85);
        imu = new MaelIMU("imu",hwMap);
        feed = MaelTellemetry.getFeed();
        tracker = new TankOdometry(dt,imu);
        intake = new MaelCollector("winch", Motor.ORBITAL20, SubsystemModels.MOTOR,hwMap);
        intake.setCollectorPowers(Companion.getINTAKE_POWER(), Companion.getOUTTAKE_POWER());
        lift = new MaelElevator("extendo", Motor.NEVEREST_NAKED,SubsystemModels.MOTOR,hwMap);
        lift.setLiftPowers(Companion.getLIFT_EXTEND(), Companion.getLIFT_RETRACT());
        limit = new MaelLimitSwitch("limit",hwMap);
        pivot = new MaelPivot(hwMap);
        pivot.setPivotPowers(Companion.getPIVOT_UP(), Companion.getPIVOT_DOWN());
        pivot.setLimit(limit);
        pivot.setFeed(feed);
        hang = new MaelCRServoSystem("hangLeftBottom","hangLeftTop","hangRightBottom","hangRightTop",hwMap);
        hang.setDirection(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
        index = new MaelServo("index",hwMap);
        hangLeftRealease = new MaelServo("hangLeftRelease",hwMap);
        hangRightRelease = new MaelServo("hangRightRelease",hwMap);
        if(startOpenCV) startOpenCV(hwMap);
        s.add(intake);
        s.add(lift);
        setSubsystemList(s);
        //hangRealease = new MaelServoSystem("hangLeftRelease","hangRightRelease",hwMap);
    }

    public void setPidConstants(){
        distancePid.setPID(Companion.getDistanceKP(), Companion.getDistanceKI(), Companion.getDistanceKD());
        turnPid.setPID(Companion.getTurnKP(), Companion.getTurnKI(), Companion.getTurnKD());
        sideTurnPid.setPID(Companion.getSideKP(), Companion.getSideKI(), Companion.getSideKD());
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
        parameters.vuforiaLicenseKey = Companion.getLICENSE_KEY();
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

    public void index(MaelController controller){
        if(controller.a()) index.setPos(.15);
        else index.setPos(.95);
    }

    public void lift(){
        hang.setPower(-.85);
    }

    public void lower(){
        hang.setPower(.85);
    }

    public void setAutoOpMode(MaelUtils.AutonomousOpMode auto){this.auto = auto;}
}
