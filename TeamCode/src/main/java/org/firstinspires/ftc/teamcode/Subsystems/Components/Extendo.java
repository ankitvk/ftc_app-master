package org.firstinspires.ftc.teamcode.Subsystems.Components;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class Extendo implements Constants {

    Hardware hardware;
    public Extendo(Hardware hardware){
        this.hardware = hardware;
    }

    public void driverControl(Gamepad gamepad){
        if(gamepad.right_bumper){
            hardware.extendo.setPower(1);
        }
        else if(gamepad.right_trigger>0){
            if(!(gamepad.right_trigger<.1)) {
                hardware.extendo.setPower(-1);
            }
        }
        else{
            hardware.extendo.setPower(0);
        }
    }

    public void stop(){
        hardware.extendo.setPower(0);
    }

    public void extend(double power){
        hardware.extendo.setPower(power);
    }

    public void retract(double power){
        hardware.extendo.setPower(-power);
    }
}
