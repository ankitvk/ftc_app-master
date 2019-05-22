package org.firstinspires.ftc.teamcode.Subsystems.Components

import com.qualcomm.robotcore.hardware.Gamepad

import org.firstinspires.ftc.teamcode.Control.Constants
import org.firstinspires.ftc.teamcode.Control.Toggle
import org.firstinspires.ftc.teamcode.Hardware.Hardware

class Intake(internal var hardware: Hardware) : Constants {
    internal var toggle: Toggle? = null

    fun intake(gamepad: Gamepad) {
        if (gamepad.left_bumper) {
            hardware.winch.power = 1.0
        } else if (gamepad.left_trigger > 0) {
            hardware.winch.power = -1.0
        } else {
            hardware.winch.power = 0.0
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

    fun index(gamepad: Gamepad) {
        when(gamepad.a){
            true -> hardware.index!!.position = .15
            false -> hardware.index!!.position = .95
        }
    }

    fun index(gamepad: Gamepad, idc: Boolean) {
        when(gamepad.dpad_up){
            true -> hardware.index!!.position = .15
            false -> hardware.index!!.position = .95
        }
    }
}
