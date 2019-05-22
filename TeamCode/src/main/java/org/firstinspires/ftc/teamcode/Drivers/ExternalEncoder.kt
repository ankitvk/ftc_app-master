package org.firstinspires.ftc.teamcode.Drivers

import com.qualcomm.robotcore.hardware.DcMotor

import org.firstinspires.ftc.teamcode.Control.Constants

class ExternalEncoder(internal var motor: DcMotor) : Constants {
    private var previousPos = 0
    private var previousTime: Long = 0
    private var rpm = 0.0
    private val power: Double = 0.0

    val position: Int
        get() = motor.currentPosition

    fun getRPM(): Double {
        val deltaPos = motor.currentPosition - previousPos
        val deltaTime = (System.nanoTime() - previousTime) / Constants.NANOSECONDS_PER_MINUTE
        if (deltaTime * 6e4 > 10) {
            rpm = deltaPos / Constants.E4T_COUNTS_PER_REV / deltaTime
            previousPos = motor.currentPosition
            previousTime = System.nanoTime()
        }
        return rpm
    }

    fun reset() {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

}
