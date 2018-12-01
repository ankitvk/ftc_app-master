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
            extendo = new SpeedControlledMotor(extensionKP,extensionKI,extensionKD,extensionMaxI),
            markerExtend1 = new SpeedControlledMotor(0,0,0,0),
            markerExtend2 = new SpeedControlledMotor(0,0,0,0),
            pivot1 = new SpeedControlledMotor(0,0,0,0),
            pivot2 = new SpeedControlledMotor(0,0,0,0),
            extension1 = new SpeedControlledMotor(0,0,0,0),
            extension2 = new SpeedControlledMotor(0,0,0,0);


    public SpeedControlledMotor[] drivetrainMotors = {frontLeft, backLeft, frontRight, backRight};

    public SpeedControlledMotor[] pivotMotors = {pivot1,pivot2};

    public SpeedControlledMotor[] extensionMotors = {extension1,extension2};

    public Drivetrain drive;

    public void init(HardwareMap hardwareMap){

        this.hwMap = hardwareMap;

        imu = new BNO055_IMU("imu",this);

        frontLeft.init(hwMap,"frontLeft");
        frontRight.init(hwMap,"frontRight");
        backLeft.init(hwMap,"backLeft");
        backRight.init(hwMap,"backRight");

        /*for(SpeedControlledMotor motor: drivetrainMotors) {
            motor.init(hwMap,motor.toString());
        }
*/
        /*for(SpeedControlledMotor motor: pivotMotors) {
            motor.init(hwMap,motor.toString());
        }

        for(SpeedControlledMotor motor: drivetrainMotors) {
            motor.init(hwMap,motor.toString());
        }*/

        extendo.init(hwMap,"extendo");

        markerExtend1.init(hwMap,"markerExtend1");
        markerExtend2.init(hwMap,"markerExtend2");


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
