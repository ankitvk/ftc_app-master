package org.firstinspires.ftc.teamcode.Subsystems.Components;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.Toggle;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class Intake implements Constants {

    Hardware hardware;
    Toggle toggle;

    public Intake(Hardware hardware){
        this.hardware = hardware;
    }

    public void intake(Gamepad gamepad){
        if(gamepad.left_bumper){
            hardware.intake.setPower(-.75);
        }
        else if(gamepad.left_trigger>0){
            hardware.intake.setPower(.75);
        }
        else{
            hardware.intake.setPower(0);
        }

        /*if(gamepad.left_trigger>0){
            hardware.intake.setPower(.75);
        }
        else if(toggle.toggle(gamepad.left_bumper)){
            hardware.intake.setPower(-.75);
        }
        else{
            hardware.intake.setPower(0);
        }*/
    }

    public void index(Gamepad gamepad){
        if(gamepad.a){
            hardware.index.setPosition(.35);
        }
        else{
            hardware.index.setPosition(.75);
        }
    }

    public void index(Gamepad gamepad,boolean idc){
        if(gamepad.dpad_up){
            hardware.index.setPosition(.35);
        }
        else{
            hardware.index.setPosition(.75);
        }
    }
}
