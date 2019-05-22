package org.firstinspires.ftc.teamcode.Control

import com.qualcomm.robotcore.hardware.Gamepad

class Controller(private val gamepad: Gamepad) : Gamepad(), Runnable {
    private var close = false
    private var aPrev = false
    private var bPrev = false
    private var xPrev = false
    private var yPrev = false
    private var leftBumperPrev = false
    private var rightBumperPrev = false
    private var rightTriggerPrev = false
    private var leftTriggerPrev = false

    val leftStickAngle: Double
        get() = Math.toDegrees(Math.atan2(leftStickY().toDouble(), (-leftStickX()).toDouble()))
    val rightStickAngle: Double
        get() = Math.toDegrees(Math.atan2(rightStickY().toDouble(), (-rightStickX()).toDouble()))

    fun a(): Boolean = gamepad.a
    fun x(): Boolean = gamepad.x
    fun y(): Boolean = gamepad.y
    fun b(): Boolean = gamepad.b

    fun aOnPress(): Boolean = a() && !aPrev
    fun bOnPress(): Boolean = b() && !bPrev
    fun yOnPress(): Boolean = y() && !yPrev
    fun xOnPress(): Boolean = x() && !xPrev

    fun leftStickX() = gamepad.left_stick_x
    fun leftStickY() = gamepad.left_stick_y
    fun rightStickX() = gamepad.right_stick_x
    fun rightStickY() = gamepad.right_stick_y

    fun dPadUp() = gamepad.dpad_up

    fun dPadDown(): Boolean {
        return gamepad.dpad_down
    }

    fun dPadLeft(): Boolean {
        return gamepad.dpad_left
    }

    fun dPadRight(): Boolean {
        return gamepad.dpad_right
    }

    fun leftBumper(): Boolean {
        return gamepad.left_bumper
    }

    fun rightBumper(): Boolean {
        return gamepad.right_bumper
    }

    fun leftBumperOnPress(): Boolean {
        return leftBumper() && !leftBumperPrev
    }

    fun rightBumperOnPress(): Boolean {
        return rightBumper() && !rightBumperPrev
    }

    fun leftStickButton(): Boolean {
        return gamepad.left_stick_button
    }

    fun rightStickButton(): Boolean {
        return gamepad.right_stick_button
    }

    fun leftTriggerPressed(): Boolean {
        return leftTrigger() > 0
    }

    fun rightTriggerPressed(): Boolean {
        return rightTrigger() > 0
    }

    fun leftTriggerOnPress(): Boolean {
        return leftTriggerPressed() && !leftTriggerPrev
    }

    fun rightTriggerOnPress(): Boolean {
        return rightTriggerPressed() && !rightTriggerPrev
    }

    fun getTan(x: Double, y: Double): Double {
        var tan = -y / x
        if (tan == java.lang.Double.NEGATIVE_INFINITY || tan == java.lang.Double.POSITIVE_INFINITY || tan != tan) tan = 0.0
        return tan
    }

    fun leftTrigger(): Float {
        return gamepad.left_trigger
    }

    fun rightTrigger(): Float {
        return gamepad.right_trigger
    }

    fun start(): Boolean {
        return gamepad.start
    }

    @Synchronized
    fun update() {
        aPrev = gamepad.a
        bPrev = gamepad.b
        xPrev = gamepad.x
        yPrev = gamepad.y
        leftBumperPrev = gamepad.left_bumper
        rightBumperPrev = gamepad.right_bumper
        rightTriggerPrev = rightTriggerPressed()
        leftTriggerPrev = leftTriggerPressed()
    }

    override fun run() {
        var close = false
        while (!close) {
            println(aPrev)
            println(a())
            update()
            close = this.close
            sleep(100)
        }
    }

    private fun sleep(sleep: Int) {
        try {
            Thread.sleep(sleep.toLong())
        } catch (e: InterruptedException) {
            e.printStackTrace()
        }

    }

    fun close() {
        close = true
    }

    fun startUpdate() {
        Thread(this).start()
    }
}

