package org.firstinspires.ftc.teamcode.Subsystems.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class Extendo implements Constants {

    Hardware hardware;
    SpeedControlledMotor extendo;

    public Extendo(Hardware hardware){

        this.hardware = hardware;
        this.extendo = hardware.extend;
    }

    private double kp = 0.01;
    private double ki = 0;
    private double kd = 0;

    public PIDController extendControl = new PIDController(kp,ki,kd,1);
    public double targetPosition;

    public void setTargetPosition() {
        targetPosition = extendo.getCurrentPosition();
    }

    public void driverControl(Gamepad gamepad){
        /*if(gamepad.right_bumper){
            hardware.extend.setPower(1);
        }
        else if(gamepad.right_trigger>0){
            if(!(gamepad.right_trigger<.1)) {
                hardware.extend.setPower(-1);
            }
        }
        else{
            hardware.extend.setPower(0);
        }*/

        kp = /*Math.abs((0.000005* -extend.getCurrentPosition()) + 0.001)*/0.001;
        if (gamepad.right_bumper) {
            extendo.setPower(-1);
        }
        else if (gamepad.right_trigger>.25) {
            extendo.setPower(1);
        }

        if (gamepad.right_bumper || gamepad.right_trigger>0) {
            targetPosition = extendo.getCurrentPosition();
        }
        else {
            double currentPosition = extendo.getCurrentPosition();
            extendo.setPower(-extendControl.power(currentPosition, targetPosition));
        }
        extendControl.setKp(kp);
    }
    private void eReset(){
        hardware.extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stop(){
        hardware.extend.setPower(0);
    }

    public void extend(double power){
        hardware.extend.setPower(power);
    }

    public void retract(double power){
        hardware.extend.setPower(-power);
    }
}
