package org.firstinspires.ftc.teamcode.Subsystems.Components

import com.qualcomm.robotcore.hardware.Gamepad

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode
import org.firstinspires.ftc.teamcode.Control.Constants
import org.firstinspires.ftc.teamcode.Control.PIDController
import org.firstinspires.ftc.teamcode.Control.Toggle
import org.firstinspires.ftc.teamcode.Hardware.Hardware
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind

class Endgame(internal var hardware: Hardware) : Constants {
    private val auto: AutonomousOpMode?
    private val telemetry: Telemetry?
    private val goldfish: GoldFind? = null

    init {
        auto = hardware.auto
        telemetry = hardware.telemetry
    }


    fun winch(gamepad: Gamepad) {
        if (gamepad.right_trigger > .75) {
            hardware.winch.power = 1.0
        } else if (gamepad.left_trigger > .75) {
            hardware.winch.power = -1.0
        } else {
            hardware.winch.power = 0.0
        }
    }

    fun lift() {
        val idc = true
        val position = 0.0
        val current = hardware.winch.currentPosition.toDouble()
        while (opModeIsActive() && hardware.winch.currentPosition < current + 22000) {
            hardware.winch.power = -1.0
            telemetry!!.addData("encoderticks: ", hardware.winch.currentPosition)

            telemetry.update()
        }
        //return position;
    }

    fun opModeIsActive() = auto!!.opModeIsActive



}
