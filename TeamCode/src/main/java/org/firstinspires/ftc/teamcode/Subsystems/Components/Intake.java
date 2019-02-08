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
            hardware.winch.setPower(1);
        }
        else if(gamepad.left_trigger>0){
            hardware.winch.setPower(-1);
        }
        else{
            hardware.winch.setPower(0);
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
            hardware.index.setPosition(.15);
        }
        else{
            hardware.index.setPosition(.95);
        }
    }

    public void index(Gamepad gamepad,boolean idc){
        if(gamepad.dpad_up){
            hardware.index.setPosition(.15);
        }
        else{
            hardware.index.setPosition(.95);
        }
    }
}
