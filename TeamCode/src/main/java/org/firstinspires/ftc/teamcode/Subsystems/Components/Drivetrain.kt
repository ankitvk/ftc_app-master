package org.firstinspires.ftc.teamcode.Subsystems.Components

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode
import org.firstinspires.ftc.teamcode.Control.Constants
import org.firstinspires.ftc.teamcode.Control.PIDController
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor
import org.firstinspires.ftc.teamcode.Hardware.Hardware
import org.firstinspires.ftc.teamcode.Drivers.BNO055_IMU

import ftc.library.MaelUtils.MaelUtils

class Drivetrain(private val hardware: Hardware) : Constants {

    private val frontLeft: SpeedControlledMotor
    private val backLeft: SpeedControlledMotor
    private val frontRight: SpeedControlledMotor
    private val backRight: SpeedControlledMotor
    private val imu: BNO055_IMU?
    private val auto: AutonomousOpMode?
    private val telemetry: Telemetry?
    private var desiredPitch = 0.0

    init {
        frontLeft = hardware.frontLeft
        backLeft = hardware.backLeft
        frontRight = hardware.frontRight
        backRight = hardware.backRight
        imu = hardware.imu
        auto = hardware.auto
        telemetry = hardware.telemetry
    }

    fun stop() {
        for (motor in hardware.drivetrainMotors) {
            motor.power = 0.0
        }
    }

    private fun eReset() {
        for (motor in hardware.drivetrainMotors) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    fun driveForward(speed: Double) {
        hardware.frontLeft.power = -speed
        hardware.backLeft.power = -speed
        hardware.frontRight.power = speed
        hardware.backRight.power = speed
    }

    fun driveForwardDistance(distance: Double) {
        eReset()
        val control = PIDController(.0007, Constants.dtKI, 0.0, 0.0)
        val ticks = distance / (Constants.WHEEL_DIAMETER * Math.PI) * Constants.DT_GEARBOX_TICKS_PER_ROTATION
        var startTime = System.nanoTime()
        var stopState: Long = 0
        while (opModeIsActive() && stopState <= 62.5) {
            val avg = hardware.frontLeft.currentPosition.toDouble()
            val power = control.power(ticks, avg)

            telemetry!!.addData("Power: ", power)
            telemetry.addData("Distance: ", ticksToDistance(avg))
            telemetry.addData("Angle: ", hardware.imu!!.yaw)
            telemetry.addLine(" ")
            telemetry.addData("error: ", control.error)
            telemetry.addData("KP*error: ", control.returnVal()[0])
            telemetry.addData("KI*i: ", control.returnVal()[1])
            telemetry.addData("KD*d: ", control.returnVal()[2])
            telemetry.update()

            hardware.frontLeft.power = -power
            hardware.backLeft.power = -power
            hardware.frontRight.power = power
            hardware.backRight.power = power

            if (Math.abs(ticks - avg) <= distanceToTicks(Constants.Companion.DISTANCE_TOLERANCE)) {
                telemetry.addData("Distance from Target: ", Math.abs(ticks - avg))
                stopState = (System.nanoTime() - startTime) / 1000000
            } else {
                startTime = System.nanoTime()
            }

            /*if(System.nanoTime()/1000000-beginTime/1000000>3000){
                break;
            }*/
        }
    }

    fun driveForTime(power: Double, time: Double) {
        eReset()
        val startTime = System.nanoTime()
        var stopState: Long = 0
        while (stopState <= time) {
            hardware.frontLeft.power = -power
            hardware.backLeft.power = -power
            hardware.frontRight.power = power
            hardware.backRight.power = power

            stopState = (System.nanoTime() - startTime) / 1000000
        }
        stop()
    }

    fun rotateForTime(power: Double, time: Double) {
        eReset()
        val startTime = System.nanoTime()
        var stopState: Long = 0
        while (stopState <= time) {
            hardware.frontLeft.power = -power
            hardware.backLeft.power = -power
            hardware.frontRight.power = -power
            hardware.backRight.power = -power

            stopState = (System.nanoTime() - startTime) / 1000000
        }
        stop()
    }

    fun rotateToAbsoluteAngle(desire: Double) {
        val controlRotate = PIDController(Constants.Companion.dtRotateKP, Constants.Companion.dtRotateKI, Constants.Companion.dtRotateKD, Constants.Companion.dtRotateMaxI) //increase Ki .00005
        val startTime = System.nanoTime()
        val stopState: Long = 0
        while (opModeIsActive()/* && (stopState <= 1000)*/) {
            val position = hardware.imu!!.getRelativeYaw()
            val power = controlRotate.power(desire, position)
            /*if(Math.abs(power)<.3){
                if(Math.abs(power)<.01){
                    break;
                }
                else if(power>=0){
                    power = .3;
                }
                else {
                    power = -.3;
                }
            }*/
            telemetry!!.addData("power", power)
            telemetry.addData("stopstate: ", stopState)
            telemetry.addData("Angle: ", hardware.imu!!.getRelativeYaw())
            telemetry.addLine(" ")
            telemetry.addData("error: ", controlRotate.error)
            telemetry.addData("KP*error: ", controlRotate.returnVal()[0])
            telemetry.addData("KI*i: ", controlRotate.returnVal()[1])
            telemetry.addData("KD*d: ", controlRotate.returnVal()[2])
            telemetry.update()
            hardware.frontLeft.power = power
            hardware.backLeft.power = power
            hardware.frontRight.power = power
            hardware.backRight.power = power

            if (Math.abs(position - desire) <= Constants.Companion.IMU_TOLERANCE) {
                /*stopState = (System.nanoTime() - startTime) / 1000000;*/
                break
            }
            /*else {
                startTime = System.nanoTime();
            }*/
            if (System.nanoTime() / 1000000 - startTime / 1000000 > 1000) {
                break
            }
        }
        stop()
    }

    fun rotateToBigAbsoluteAngle(desire: Double) {
        val controlRotate = PIDController(Constants.Companion.dtBigRotateKP, Constants.Companion.dtBigRotateKI, Constants.Companion.dtBigRotateKD, Constants.Companion.dtBigRotateMaxI)
        var startTime = System.nanoTime()
        val beginTime = startTime

        var stopState: Long = 0
        while (opModeIsActive() && stopState <= 1000) {
            val position = hardware.imu!!.getRelativeYaw()
            val power = controlRotate.power(desire, position)
            /*if(Math.abs(power)<.35){
                if(Math.abs(power)<.01){
                    break;
                }
                else if(power>=0){
                    power = .35;
                }
                else {
                    power = -.35;
                }
            }*/
            telemetry!!.addData("stopstate: ", stopState)
            telemetry.addData("Angle: ", hardware.imu!!.getRelativeYaw())
            telemetry.addLine(" ")
            telemetry.addData("KP*error: ", controlRotate.returnVal()[0])
            telemetry.addData("KI*i: ", controlRotate.returnVal()[1])
            telemetry.addData("KD*d: ", controlRotate.returnVal()[2])
            telemetry.update()
            hardware.frontLeft.power = power
            hardware.backLeft.power = power
            hardware.frontRight.power = power
            hardware.backRight.power = power

            if (Math.abs(Math.abs(position) - Math.abs(desire)) <= Constants.Companion.IMU_TOLERANCE) {
                stopState = (System.nanoTime() - startTime) / 1000000
            } else {
                startTime = System.nanoTime()
            }
            /*if(System.nanoTime()/1000000-beginTime/1000000>1500){
                break;
            }*/
        }
        stop()
    }

    fun rotate(speed: Double) {
        hardware.frontLeft.power = speed
        hardware.backLeft.power = speed
        hardware.frontRight.power = speed
        hardware.backRight.power = speed
    }

    fun opModeIsActive(): Boolean {
        return auto!!.opModeIsActive
    }

    private fun distanceToTicks(distance: Double): Double {
        return distance / (Constants.Companion.WHEEL_DIAMETER * Math.PI) * Constants.Companion.DT_GEARBOX_TICKS_PER_ROTATION
    }

    private fun ticksToDistance(ticks: Double): Double {
        return ticks * (Constants.Companion.WHEEL_DIAMETER * Math.PI) / Constants.Companion.DT_GEARBOX_TICKS_PER_ROTATION
    }

    fun rotateToRelativeAngle(degrees: Double) {
        rotateToAbsoluteAngle(hardware.imu!!.yaw + degrees)
    }

    fun drive(gamepad: Gamepad) {
        val pitchCorrection = PIDController(.02, 0.0, 0.0, 0.0)
        val antiTipPower: Double = if (Math.abs(desiredPitch - imu!!.pitch) > 2) pitchCorrection.power(desiredPitch, imu.pitch) else 0.0
        val yDirection = gamepad.left_stick_y.toDouble()
        val xDirection = gamepad.right_stick_x.toDouble()
        var speedReducer = 1.0

        val leftPower = (yDirection - xDirection) * Constants.Companion.SPEED_MULTIPLIER
        val rightPower = (-yDirection - xDirection) * Constants.Companion.SPEED_MULTIPLIER

        if (gamepad.a) {
            speedReducer = 0.25
        } else {
            speedReducer = 1.0
        }
        hardware.backLeft.power = leftPower * speedReducer + antiTipPower
        hardware.frontLeft.power = leftPower * speedReducer + antiTipPower
        hardware.backRight.power = rightPower * speedReducer - antiTipPower
        hardware.frontRight.power = rightPower * speedReducer - antiTipPower
    }

    fun mecanum(gamepad: Gamepad) {

        val leftY = gamepad.left_stick_y.toDouble()
        val leftX = gamepad.left_stick_x.toDouble()
        val rightX = gamepad.right_stick_x.toDouble()
        val x = -leftY
        val angle = Math.atan2(leftX, x)
        val adjustedAngle = angle + Math.PI / 4
        val speedMagnitude = Math.hypot(x, leftX)

        val speeds = doubleArrayOf(Math.sin(adjustedAngle), Math.cos(adjustedAngle), Math.cos(adjustedAngle), Math.sin(adjustedAngle))

        MaelUtils.normalizeValues(speeds)

        speeds[0] = speeds[0] * speedMagnitude + rightX
        speeds[1] = speeds[1] * speedMagnitude + rightX
        speeds[2] = -speeds[2] * speedMagnitude + rightX
        speeds[3] = -speeds[3] * speedMagnitude + rightX

        frontLeft.power = speeds[0]
        backLeft.power = speeds[1]
        frontRight.power = speeds[2]
        backRight.power = speeds[3]

        telemetry!!.addData("FL:", frontLeft.power)
        telemetry.addData("BL:", backLeft.power)
        telemetry.addData("FR:", frontRight.power)
        telemetry.addData("BR:", backRight.power)
        telemetry.addData("imu:", imu!!.yaw)
        telemetry.update()

    }

    fun resetDesiredPitch() {
        desiredPitch = imu!!.pitch
    }

    fun controlDrive(gamepad: Gamepad) {
        val yDirection = gamepad.left_stick_y.toDouble()
        val xDirection = gamepad.right_stick_x.toDouble()

        val leftPower = (yDirection - xDirection) * Constants.Companion.SPEED_MULTIPLIER
        val rightPower = (-yDirection - xDirection) * Constants.Companion.SPEED_MULTIPLIER
        hardware.frontLeft.setSpeed(leftPower)
        hardware.backLeft.power = hardware.frontLeft.power
        hardware.frontRight.setSpeed(rightPower)
        hardware.backRight.power = hardware.frontRight.power

    }

    fun driveForwardDistancePOM(distance: Double) {
        eReset()
        val control = PIDController(-0.0001, 0.000001, 0.0, 1.0)
        val ticks = distance / (Constants.Companion.WHEEL_DIAMETER * Math.PI) * Constants.Companion.DT_GEARBOX_TICKS_PER_ROTATION
        var startTime = System.nanoTime()
        val beginTime = startTime
        var stopState: Long = 0
        while (opModeIsActive() && stopState <= 250) {
            val avg = hardware.frontLeft.currentPosition.toDouble()
            val power = control.pom(ticks, avg, ticks)
            telemetry!!.addData("Power: ", power)
            telemetry.addData("Distance: ", ticksToDistance(avg))
            telemetry.addData("Angle: ", hardware.imu!!.yaw)
            telemetry.addLine(" ")
            telemetry.addData("error: ", control.error)
            telemetry.addData("KP*deltaDistance: ", control.kp * ticks)
            telemetry.addData("KI*i: ", control.returnVal()[1])
            telemetry.addData("KD*d: ", control.returnVal()[2])
            telemetry.update()

            hardware.frontLeft.setSpeed(-power)
            hardware.backLeft.power = hardware.frontLeft.power
            hardware.frontRight.setSpeed(power)
            hardware.backRight.power = hardware.frontRight.power

            if (Math.abs(ticks - avg) <= distanceToTicks(Constants.Companion.DISTANCE_TOLERANCE)) {
                telemetry.addData("Distance from Target: ", Math.abs(ticks - avg))
                stopState = (System.nanoTime() - startTime) / 1000000
            } else {
                startTime = System.nanoTime()
            }
        }
    }


}
