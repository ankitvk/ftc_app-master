package org.firstinspires.ftc.teamcode.Subsystems.Components

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad

import org.firstinspires.ftc.teamcode.Control.Constants
import org.firstinspires.ftc.teamcode.Control.PIDController
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor
import org.firstinspires.ftc.teamcode.Hardware.Hardware

class Extendo(internal var hardware: Hardware) : Constants {
    internal var extendo: SpeedControlledMotor

    private var kp = 0.01
    private val ki = 0.0
    private val kd = 0.0

    var extendControl = PIDController(kp, ki, kd, 1.0)
    var targetPosition: Double = 0.toDouble()

    init {
        this.extendo = hardware.extend
    }

    fun setTargetPosition() {
        targetPosition = extendo.currentPosition.toDouble()
    }

    fun driverControl(gamepad: Gamepad) {
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

        kp = /*Math.abs((0.000005* -extend.getCurrentPosition()) + 0.001)*/ 0.001

        if (gamepad.right_bumper) extendo.power = -1.0
         else if (gamepad.right_trigger > .25) extendo.power = 1.0
        if (gamepad.right_bumper || gamepad.right_trigger > 0) targetPosition = extendo.currentPosition.toDouble()
         else {
            val currentPosition = extendo.currentPosition.toDouble()
            extendo.power = -extendControl.power(currentPosition, targetPosition)
        }
        extendControl.kp = kp
    }

    private fun eReset() {
        hardware.extend.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hardware.extend.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun stop() {
        hardware.extend.power = 0.0
    }

    fun extend(power: Double) {
        hardware.extend.power = power
    }

    fun retract(power: Double) {
        hardware.extend.power = -power
    }
}
