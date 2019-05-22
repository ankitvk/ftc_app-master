package org.firstinspires.ftc.teamcode.Control

import com.qualcomm.robotcore.util.Range

class PIDController : Constants {

    var i = 0.0
        private set
    private var d: Double = 0.0
    var error: Double = 0.0
        private set
    private var power: Double = 0.0
    var kp: Double = 0.0
    var ki: Double = 0.0
    var kd: Double = 0.0
    private var previousError = 0.0
    private var maxI: Double = 0.0
    private var previousTime = 0.0
    var current: Double = 0.0
        private set
    var target: Double = 0.0
        private set
    private var minPower = 0.0


    constructor(KP: Double, KI: Double, KD: Double, maxI: Double, minPower: Double) {
        this.kp = KP
        this.ki = KI
        this.kd = KD
        this.maxI = maxI
        this.minPower = minPower
    }

    constructor(KP: Double, KI: Double, KD: Double, maxI: Double) {
        this.kp = KP
        this.ki = KI
        this.kd = KD
        this.maxI = maxI
    }


    fun power(lock: Double, currentLoc: Double): Double {

        current = currentLoc
        target = lock
        error = lock - currentLoc
        val deltaTime = (System.nanoTime() - previousTime) / Constants.NANOSECONDS_PER_MINUTE
        if (Math.abs(currentLoc) > Math.abs(lock) * .75) {
            i += error * deltaTime
        }
        d = (error - previousError) / deltaTime
        power = kp * error + ki * i + kd * d
        power = if (power < 0) Math.min(power, -minPower) else Math.max(power, minPower)
        previousTime = System.nanoTime().toDouble()
        previousError = error
        return power
    }

    fun pom(lock: Double, currentLoc: Double, deltaProcessVariable: Double): Double {
        current = currentLoc
        target = lock
        error = lock - currentLoc
        val deltaTime = (System.nanoTime() - previousTime) / Constants.NANOSECONDS_PER_MINUTE
        i += error * deltaTime
        d = (error - previousError) / deltaTime
        power = kp * deltaProcessVariable + ki * i + kd * d
        previousTime = System.nanoTime().toDouble()
        previousError = error
        power = if (power < 0) Math.min(power, -minPower) else Math.max(power, minPower)
        return power
    }

    fun reset() {
        i = 0.0
        previousError = 0.0
    }

    fun returnVal(): DoubleArray {
        return doubleArrayOf(kp * error, ki * i, kd * d, power)
    }


}
