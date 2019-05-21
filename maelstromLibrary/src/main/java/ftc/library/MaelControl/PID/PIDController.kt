package ftc.library.MaelControl.PID

import ftc.library.MaelSensors.MaelTimer
import ftc.library.MaelUtils.LibConstants

/*class for PID controller*/
class PIDController : LibConstants {
    var integral: Double = 0.0
        private set
    private var d: Double = 0.0
    private var error: Double = 0.0
    private var power: Double = 0.0
    var kp: Double = 0.0
    var ki: Double = 0.00
    var kd: Double = 0.0
    private var previousError: Double = 0.0
    private var maxI: Double = 0.0
    private var previousTime = 0.0
    private val loopTimer = MaelTimer()

    val P: Double
        get() = kp * error
    val I: Double
        get() = ki * integral
    val D: Double
        get() = kd*d

    constructor(KP: Double, KI: Double, KD: Double, maxI: Double) {
        this.kp = KP
        this.ki = KI
        this.kd = KD
        this.maxI = maxI
    }

    constructor(KP: Double, KI: Double, KD: Double) {
        this.kp = KP
        this.ki = KI
        this.kd = KD
        maxI = 0.0
    }

    constructor(KP: Double, KI: Double) {
        this.kp = KP
        this.ki = KI
        maxI = 0.0
    }

    fun power(target: Double, currentLoc: Double): Double {
        val error = target - currentLoc
        this.error = error
        val deltaTime = (System.nanoTime() - previousTime) / LibConstants.NANOSECS_PER_MIN/*loopTimer.milliSecs()*/
        //loopTimer.reset();
        if (Math.abs(currentLoc) > Math.abs(target) * 0.7){
        integral +=  error * deltaTime}
        else integral = 0.0
        if (maxI != 0.0) integral = Math.min(maxI, Math.max(-maxI, integral))
        d = (error - previousError) / deltaTime
        previousTime = System.nanoTime().toDouble()
        previousError = error
        //power = kp * error + ki * integral + kd * d
        power = P + I + D
        return power
    }

    fun returnVal(): DoubleArray {
        return doubleArrayOf(kp * error, ki * integral, kd * d, power)
    }

    fun getError() = Math.abs(error)

    fun setPID(kp: Double, ki: Double, kd: Double) {
        this.kp = kp
        this.ki = ki
        this.kd = kd
    }

    fun reset() {
        integral = 0.0
        previousError = 0.0
    }


}