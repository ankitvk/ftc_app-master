package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;

public class Hardware implements Constants {

    HardwareMap hwMap;

    public AutonomousOpMode auto;

    public Telemetry telemetry;

    public BNO055_IMU imu;

    public SpeedControlledMotor
            frontLeft = new SpeedControlledMotor(dtKP,dtKI,dtKD,dtMaxI),
            frontRight = new SpeedControlledMotor(dtKP,dtKI,dtKD,dtMaxI),
            backLeft = new SpeedControlledMotor(dtKP,dtKI,dtKD,dtMaxI),
            backRight = new SpeedControlledMotor(dtKP,dtKI,dtKD,dtMaxI);


    public SpeedControlledMotor[] drivetrainMotors = {frontLeft, backLeft, frontRight, backRight};

    public void setAuto (AutonomousOpMode auto, Telemetry telemetry) {
        this.auto = auto;
        this.telemetry = telemetry;
    }

    public HardwareMap getHwMap() {
        return hwMap;
    }


}
