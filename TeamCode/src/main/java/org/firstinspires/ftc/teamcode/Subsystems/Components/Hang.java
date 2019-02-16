package org.firstinspires.ftc.teamcode.Subsystems.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class Hang implements Constants {

    private CRServo hangLeftTop,hangLeftBottom,hangRightTop,hangRightBottom;

    private Servo hangLeftRelease,hangRightRelease;

    double servoReleasePos = 0.7;

    public Hang(Hardware hardware){

        this.hangLeftTop = hardware.hangLeftTop;
        this.hangLeftBottom = hardware.hangLeftBottom;
        this.hangRightTop = hardware.hangRightTop;
        this.hangRightBottom = hardware.hangRightBottom;

        this.hangLeftRelease = hardware.hangLeftRelease;
        this.hangRightRelease = hardware.hangRightRelease;
    }

    private void release(){
       hangLeftRelease.setPosition(servoReleasePos);
       hangRightRelease.setPosition(1-servoReleasePos);
       setPower(1);
    }

    private void setPower(double power){
        hangLeftBottom.setPower(power*.75);
        hangRightTop.setPower(power*.75);
        hangLeftTop.setPower(power*-.75);
        hangRightBottom.setPower(power*-.75);
    }


}
