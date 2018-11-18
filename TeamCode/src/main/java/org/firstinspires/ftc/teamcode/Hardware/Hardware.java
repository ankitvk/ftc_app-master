package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

public class Hardware implements Constants {

    HardwareMap hwMap;

    public AutonomousOpMode auto;

    public Telemetry telemetry;

    public Servo hook;

    public Servo marker;

    public BNO055_IMU imu;

    public SpeedControlledMotor
            frontLeft = new SpeedControlledMotor(dtKP,dtKI,dtKD,dtMaxI),
            frontRight = new SpeedControlledMotor(dtKP,dtKI,dtKD,dtMaxI),
            backLeft = new SpeedControlledMotor(dtKP,dtKI,dtKD,dtMaxI),
            backRight = new SpeedControlledMotor(dtKP,dtKI,dtKD,dtMaxI),
            extendo = new SpeedControlledMotor(extensionKP,extensionKI,extensionKD,extensionMaxI);


    public SpeedControlledMotor[] drivetrainMotors = {frontLeft, backLeft, frontRight, backRight};

    public Drivetrain drive;

    public void init(HardwareMap hardwareMap){

        this.hwMap = hardwareMap;

        imu = new BNO055_IMU("imu",this);

        frontLeft.init(hwMap,"frontLeft");
        frontRight.init(hwMap,"frontRight");
        backLeft.init(hwMap,"backLeft");
        backRight.init(hwMap,"backRight");

        extendo.init(hwMap,"extendo");

        hook = hardwareMap.servo.get("hook");

        marker = hardwareMap.servo.get("marker");

        drive = new Drivetrain(this);

    }

    public void setAuto (AutonomousOpMode auto, Telemetry telemetry) {
        this.auto = auto;
        this.telemetry = telemetry;
    }

    public HardwareMap getHwMap() {
        return hwMap;
    }


}