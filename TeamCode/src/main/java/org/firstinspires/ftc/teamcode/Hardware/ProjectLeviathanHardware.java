package org.firstinspires.ftc.teamcode.Hardware;

import android.provider.SyncStateContract;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

public class ProjectLeviathanHardware implements Constants {
    HardwareMap hwMap;

    public AutonomousOpMode auto;

    public Telemetry telemetry;

    public SpeedControlledMotor
            frontLeft = new SpeedControlledMotor(dtKP,dtKI,dtKD,dtMaxI),
            frontRight = new SpeedControlledMotor(dtKP,dtKI,dtKD,dtMaxI),
            backLeft = new SpeedControlledMotor(dtKP,dtKI,dtKD,dtMaxI),
            backRight = new SpeedControlledMotor(dtKP,dtKI,dtKD,dtMaxI);


    public SpeedControlledMotor[] drivetrainMotors = {frontLeft, backLeft, frontRight, backRight};

    public Drivetrain drive;

    public BNO055_IMU imu;


    public void init(HardwareMap hardwareMap){

        this.hwMap = hardwareMap;

        frontLeft.init(hwMap,"frontLeft");
        frontRight.init(hwMap,"frontRight");
        backLeft.init(hwMap,"backLeft");
        backRight.init(hwMap,"backRight");

        //imu = new BNO055_IMU("imu",this);


    }

    public void setAuto (AutonomousOpMode auto, Telemetry telemetry) {
        this.auto = auto;
        this.telemetry = telemetry;
    }

    public HardwareMap getHwMap() {
        return hwMap;
    }


}
