package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;

public class DummyHardware implements Constants {

    HardwareMap hwMap;

    public AutonomousOpMode auto;

    public Telemetry telemetry;

    //public Servo led;

    public RevBlinkinLedDriver led;

    public SpeedControlledMotor
            winch = new SpeedControlledMotor(dtKP,dtKI,dtKD,dtMaxI);


    public void init(HardwareMap hardwareMap){

        this.hwMap = hardwareMap;

        winch.init(hwMap,"frontLeft");
    }
    public void setAuto (AutonomousOpMode auto, Telemetry telemetry) {
        this.auto = auto;
        this.telemetry = telemetry;
    }

    public HardwareMap getHwMap() {
        return hwMap;
    }
}
