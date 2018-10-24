package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Vuforia;

//import org.firstinspires.ftc.robotcore.external.navigation.R
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;/*use*/
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;
import org.firstinspires.ftc.teamcode.Sensors.ExternalEncoder;

public class Hardware implements Constants
{
    HardwareMap hwMap;

    public SpeedControlledMotor frontLeft = new SpeedControlledMotor(frontLeftKP,frontLeftKI,frontLeftKD,frontLeftMaxI),
                                frontRight = new SpeedControlledMotor(frontRightKP,frontRightKI,frontRightKD,frontRightMaxI),
                                backLeft = new SpeedControlledMotor(backLeftKP,backLeftKI,backLeftKD,backLeftMaxI),
                                backRight = new SpeedControlledMotor(backRightKP,backRightKI,backRightKD,backRightMaxI),
                                topPivot = new SpeedControlledMotor(topPivotKP,topPivotKI,topPivotKD,topPivotMaxI),
                                extensionLeft = new SpeedControlledMotor(extensionLeftKP,extensionLeftKI,extensionLeftKD,extensionLeftMaxI),
                                extensionRight = new SpeedControlledMotor(extensionRightKP,extensionRightKI,extensionRightKD,extensionRightMaxI),
                                intake = new SpeedControlledMotor(intakeKP,intakeKI,intakeKD,intakeMaxI);

    public BNO055_IMU imu;

    public ExternalEncoder pivotCount;

    public DcMotor pivot;

    public CRServo LEDStrip;


    public void init(HardwareMap hardwareMap){

        this.hwMap = hardwareMap;

        imu = new BNO055_IMU("imu",this);

        frontLeft.init(hwMap,"frontLeft");
        frontRight.init(hwMap,"frontRight");
        backLeft.init(hwMap,"backLeft");
        backRight.init(hwMap,"backRight");
        topPivot.init(hwMap,"topPivot");
        extensionLeft.init(hwMap,"extensionLeft");
        extensionRight.init(hwMap,"extensionRight");
        intake.init(hwMap,"intake");

        pivotCount = new ExternalEncoder(pivot);

        LEDStrip = hwMap.crservo.get("LEDStrip");



    }
}
