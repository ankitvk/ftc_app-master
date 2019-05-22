package org.firstinspires.ftc.teamcode.Subsystems.Components

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode
import org.firstinspires.ftc.teamcode.Control.Constants
import org.firstinspires.ftc.teamcode.Hardware.Hardware

class Hang(internal var hardware: Hardware) : Constants {

    private val auto: AutonomousOpMode?
    private val telemetry: Telemetry?
    private val g: Gamepad? = null

    private val hangLeftTop: CRServo?
    private val hangLeftBottom: CRServo?
    private val hangRightTop: CRServo?
    private val hangRightBottom: CRServo?

    private val hangLeftRelease: Servo?
    private val hangRightRelease: Servo?

    init {
        hangLeftTop = hardware.hangLeftTop
        hangLeftBottom = hardware.hangLeftBottom
        hangRightTop = hardware.hangRightTop
        hangRightBottom = hardware.hangRightBottom

        hangLeftRelease = hardware.hangLeftRelease
        hangRightRelease = hardware.hangRightRelease

        auto = hardware.auto
        telemetry = hardware.telemetry
    }


    fun lift(gamepad: Gamepad) {
        //this.g = gamepad
        val power: Double

        if (gamepad.right_trigger > 0) power = gamepad.right_trigger.toDouble()
        else if (gamepad.left_trigger > 0) power = (-gamepad.left_trigger).toDouble()
         else power = 0.0

        hardware.hangRightTop!!.power = .85 * power
        hardware.hangRightBottom!!.power = -.85 * power

        hardware.hangLeftTop!!.power = -.85 * power
        hardware.hangLeftBottom!!.power = .85 * power
    }

    fun drop() {
        hangLeftBottom!!.power = -.85
        hangRightTop!!.power = -.85
        hangLeftTop!!.power = .85
        hangRightBottom!!.power = .85

        sleep(250)

        hangLeftRelease!!.position = .75
        hangRightRelease!!.position = .25

        val startTime = System.nanoTime()
        var stopState: Long = 0
        while (stopState <= 3500 && opModeIsActive()) {
            hangLeftBottom.power = .85
            hangRightTop.power = .85
            hangLeftTop.power = -.85
            hangRightBottom.power = -.85

            stopState = (System.nanoTime() - startTime) / 1000000
        }
        stop()
    }

    fun sleep(milliseconds: Long) {
        try {
            Thread.sleep(milliseconds)
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        }

    }

    private fun stop() {
        for(motor in hardware.theHangGang) motor!!.power = 0.0
    }

    fun opModeIsActive(): Boolean {
        return auto!!.opModeIsActive
    }


}
