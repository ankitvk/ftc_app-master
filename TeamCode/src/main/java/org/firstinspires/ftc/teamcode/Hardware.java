package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;
import org.firstinspires.ftc.teamcode.Sensors.ExternalEncoder;

public class Hardware implements Constants
{
    HardwareMap hwMap;

    public AutonomousOpMode auto;

    public SpeedControlledMotor frontLeft = new SpeedControlledMotor(frontLeftKP,frontLeftKI,frontLeftKD,frontLeftMaxI),
                                frontRight = new SpeedControlledMotor(frontRightKP,frontRightKI,frontRightKD,frontRightMaxI),
                                backLeft = new SpeedControlledMotor(backLeftKP,backLeftKI,backLeftKD,backLeftMaxI),
                                backRight = new SpeedControlledMotor(backRightKP,backRightKI,backRightKD,backRightMaxI),
                                pivot1 = new SpeedControlledMotor(topPivot1KP,topPivot1KI,topPivot1KD,topPivot1MaxI),
                                pivot2 = new SpeedControlledMotor(topPivot2KP,topPivot2KI,topPivot2KD,topPivot2MaxI),
                                extensionLeft = new SpeedControlledMotor(extensionKP,extensionKI,extensionKD,extensionMaxI),
                                extensionRight = new SpeedControlledMotor(extensionKP,extensionKI,extensionKD,extensionMaxI);

    public BNO055_IMU imu;

    public Servo stopRotation,stopExtension,indexer;

    public CRServo intake;

    public DigitalChannel magLimitSwitch;

    //public ExternalEncoder pivotCount;

    //public DcMotor pivot;

    //public CRServo LEDStrip;


    public void init(HardwareMap hardwareMap){

        this.hwMap = hardwareMap;

        imu = new BNO055_IMU("imu",this);

        frontLeft.init(hwMap,"frontLeft");
        frontRight.init(hwMap,"frontRight");
        backLeft.init(hwMap,"backLeft");
        backRight.init(hwMap,"backRight");
        pivot1.init(hwMap,"pivot1");
        pivot2.init(hwMap,"pivot2");
        extensionLeft.init(hwMap,"extensionLeft");
        extensionRight.init(hwMap,"extensionRight");

        stopRotation = hwMap.servo.get("stopRotation");
        stopExtension = hwMap.servo.get("stopExtension");
        indexer = hwMap.servo.get("indexer");
        intake = hwMap.crservo.get("intake");

        magLimitSwitch = hwMap.digitalChannel.get("magLimitSwitch");

        //pivotCount = new ExternalEncoder(pivot);

        //LEDStrip = hwMap.crservo.get("LEDStrip");

    }

    public HardwareMap getHwMap() {
        return hwMap;
    }
}
