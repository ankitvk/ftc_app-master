package org.firstinspires.ftc.teamcode.Control

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class SpeedControlledMotor(KP: Double, KI: Double, KD: Double, maxI: Double) : Constants {
    private var motor: DcMotor? = null
    private var previousPos = 0
    private var previousTime: Long = 0
    private var rpm = 0.0

    internal var PIDController: PIDController

    var rpmTemp = 0.0
        internal set

    val currentPosition: Int
        get() = motor!!.currentPosition

    var power: Double
        get() = motor!!.power
        set(power) {
            motor!!.power = power
        }

    var mode: DcMotor.RunMode
        get() = motor!!.mode
        set(runMode){
            motor!!.mode = runMode
        }

    init {
        this.PIDController = PIDController(KP, KI, KD, maxI)
    }


    fun init(hardwareMap: HardwareMap, name: String) {
        motor = hardwareMap.dcMotor.get(name)
    }

/*    fun setMode(runMode: DcMotor.RunMode) {
        motor!!.mode = runMode
    }*/

    fun getRPM(): Double {
        val deltaPos = motor!!.currentPosition - previousPos
        val deltaTime = (System.nanoTime() - previousTime) / Constants.NANOSECONDS_PER_MINUTE
        if (deltaTime * 6e4 > 10) {
            rpm = deltaPos / Constants.DT_GEARBOX_TICKS_PER_ROTATION / deltaTime
            previousPos = motor!!.currentPosition
            previousTime = System.nanoTime()
        }
        return rpm
    }

    fun setSpeed(speed: Double) {
        val rpm = Constants.DT_MAX_RPM * speed
        rpmTemp = rpm
        val power = PIDController.power(rpm, getRPM())
        motor!!.power = power
    }

    fun setRPM(rpm: Double) {
        val power = PIDController.power(rpm, getRPM())
        motor!!.power = power

    }

    fun setZeroPowerBehavior(behavior: DcMotor.ZeroPowerBehavior) {
        motor!!.zeroPowerBehavior = behavior
    }

    fun getAngle(ticksPerRotation: Double): Double {
        return 360 * motor!!.currentPosition / ticksPerRotation
    }

    fun setReverseMode() {
        motor!!.direction = DcMotorSimple.Direction.REVERSE
    }

    fun setForwardMode() {
        motor!!.direction = DcMotorSimple.Direction.FORWARD
    }


}
