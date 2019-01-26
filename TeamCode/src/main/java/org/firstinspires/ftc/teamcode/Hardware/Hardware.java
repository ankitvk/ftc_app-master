package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Endgame;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Extendo;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Crater.craterLeft;
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Crater.craterMiddle;
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Crater.craterRight;
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Depot.depotLeft;
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Depot.depotMiddle;
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Depot.depotRight;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Pivot;

public class Hardware implements Constants {

    HardwareMap hwMap;

    public AutonomousOpMode auto;

    public Telemetry telemetry;

    public Servo /*hookRelease,hookSwivel,drop,*/index,marker;

    public CRServo intake;

    public BNO055_IMU imu;

    public DigitalChannel limit;

    //public Servo led;

    //public RevBlinkinLedDriver led;

    double kp = 0.00015;
    double ki = 0;

    public SpeedControlledMotor
            frontLeft = new SpeedControlledMotor(.0031,0,0,1),
            backLeft = new SpeedControlledMotor(.0031,0,0,1),
            frontRight = new SpeedControlledMotor(.0031,0,0,1),
            backRight = new SpeedControlledMotor(.0031,0,0,1),
            extendo = new SpeedControlledMotor(extensionKP,extensionKI,extensionKD,extensionMaxI),
            winch = new SpeedControlledMotor(0,0,0,1),
            pivot1 = new SpeedControlledMotor(0,0,0,0),
            pivot2 = new SpeedControlledMotor(0,0,0,0);

    public SpeedControlledMotor[] drivetrainMotors = {frontLeft, backLeft, frontRight, backRight};

    public SpeedControlledMotor[] pivotMotors = {pivot1,pivot2};


    public Drivetrain drive;
    public Pivot pivot ;
    public Extendo extendoo;
    public Endgame endgame;

    public depotLeft depotLeft;
    public depotMiddle depotMiddle;
    public depotRight depotRight;

    public craterLeft craterLeft;
    public craterMiddle craterMiddle;
    public craterRight craterRight;

    public void init(HardwareMap hardwareMap){

        this.hwMap = hardwareMap;

        imu = new BNO055_IMU("imu",this);

        frontLeft.init(hwMap,"frontLeft");
        frontRight.init(hwMap,"frontRight");
        backLeft.init(hwMap,"backLeft");
        backRight.init(hwMap,"backRight");

        //led = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        extendo.init(hwMap,"extendo");

        pivot1.init(hwMap,"pivot1");
        pivot2.init(hwMap,"pivot2");

        winch.init(hwMap,"winch");

        //hookRelease = hardwareMap.servo.get("hookRelease");
        //hookSwivel = hardwareMap.servo.get("hookSwivel");

        marker = hardwareMap.servo.get("marker");

        //drop = hardwareMap.servo.get("drop");

        index = hardwareMap.servo.get("index");

        intake = hardwareMap.crservo.get("intake");

        drive = new Drivetrain(this);
        pivot = new Pivot(this);
        extendoo = new Extendo(this);
        endgame = new Endgame(this);

        depotLeft = new depotLeft(this);
        depotMiddle = new depotMiddle(this);
        depotRight = new depotRight(this);

        limit = hardwareMap.digitalChannel.get("limit");
    }

    public void setAuto (AutonomousOpMode auto, Telemetry telemetry) {
        this.auto = auto;
        this.telemetry = telemetry;
    }

    public HardwareMap getHwMap() {
        return hwMap;
    }


}
