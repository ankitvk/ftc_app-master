package org.firstinspires.ftc.teamcode.Subsystems.Components

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode
import org.firstinspires.ftc.teamcode.Control.Constants
import org.firstinspires.ftc.teamcode.Control.Controller
import org.firstinspires.ftc.teamcode.Control.PIDController
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor
import org.firstinspires.ftc.teamcode.Hardware.Hardware

class Pivot(internal var hardware: Hardware) : Constants {

    private var kp = 0.01
    private var ki = 0.0
    private var kd = 0.0

    private var targetPosition: Double = 0.toDouble()
    private val basePower = 1.0
    private val baseDownPower = 1.0
    private var rotatorPower = basePower
    private var downPower = -0.1

    private val isReset = false
    private var scoringPosition = false


    var pivotControl = PIDController(kp, ki, kd, 1.0)

    private var liftPosition = 0.0
    private val auto: AutonomousOpMode
    private val telemetry: Telemetry

    val position: Double
        get() = hardware.pivot1.currentPosition.toDouble()

    val rawPower: Double
        get() = hardware.pivotMotors[0].power

    var angle: Double
        get() = Constants.PIVOT_LIMIT_ANGLE + hardware.pivot1.getAngle(Constants.PIVOT_TICKS_PER_ROTATION)
        set(angle) {
            val pivotAngleController = PIDController(Math.abs(Math.cos(angle)) * 0.003 + 0.05, 0.0, 0.0, 0.0)

            val power = pivotAngleController.power(angle, angle)

            hardware.pivot1.power = power
            hardware.pivot2.power = power

        }

    init {
        auto = hardware.auto
        telemetry = hardware.telemetry
    }

    fun driverControl(controller: Gamepad) {
        kp = Constants.pivotKP * -liftPosition + 0.001
        ki = 0.0
        kd = 0.0
        rotatorPower = 0.0001 * -liftPosition + basePower
        downPower = 0.001 * Math.abs(position) + baseDownPower
        if (controller.left_bumper) {
            setPower(-rotatorPower)
        } else if (controller.right_bumper) {
            setPower(downPower)
        } else if (scoringPosition) {
            angle = 135.0
        } else {
            setPower(0.0)
        }

        if (controller.right_bumper || controller.left_bumper) {
            targetPosition = position
            scoringPosition = false
        } else {
            if (controller.dpad_right) {
                scoringPosition = true
            }
            val currentPosition = position
            //setPower(pivotControl.power(targetPosition,currentPosition));
        }

        if (!hardware.limit.state && !isReset) {
            eReset()
        }

        pivotControl.kp = kp
        pivotControl.ki = ki
        pivotControl.kd = kd
    }

    fun driverControlSingle(controller: Gamepad) {
        kp = Constants.pivotKP * -liftPosition + 0.001
        ki = 0.0
        kd = 0.0
        rotatorPower = 0.0005 * -liftPosition + basePower
        downPower = 0.005 * Math.abs(position) + baseDownPower
        if (controller.dpad_right) {
            setPower(-rotatorPower)
        } else if (controller.dpad_left) {
            setPower(downPower)
        } else {
            setPower(0.0)
        }

        if (controller.dpad_left || controller.dpad_right)
            targetPosition = position
        else {
            val currentPosition = position
            //setPower(pivotControl.power(targetPosition,currentPosition));
        }

        if (!hardware.limit.state && !isReset) {
            eReset()
        }

        pivotControl.kp = kp
        pivotControl.ki = ki
        pivotControl.kd = kd
    }

    fun scoringPosition() {
        val scoringPosition = ftc.library.MaelControl.PID.PIDController(1.0, 0.0, 0.0, 0.0)
        var startTime = System.nanoTime()
        val beginTime = startTime
        var stopState: Long = 0

        val ticks = 0.0

        while (opModeIsActive() && stopState <= 250) {
            val position = position
            val power = scoringPosition.power(ticks, position)

            telemetry.addData("stopstate: ", stopState)
            telemetry.addData("KP*error: ", scoringPosition.P)
            telemetry.addData("KI*i: ", scoringPosition.I)
            telemetry.addData("KD*d: ", scoringPosition.D)
            telemetry.update()

            hardware.pivot1.power = power
            hardware.pivot2.power = power

            if (!hardware.limit.state) {
                stopState = (System.nanoTime() - startTime) / 1000000
            } else {
                startTime = System.nanoTime()
            }
        }
        stop()
        eReset()
    }

    fun downPosition() {
        val scoringPosition = PIDController(1.0, 0.0, 0.0, 0.0)
        var startTime = System.nanoTime()
        val beginTime = startTime
        var stopState: Long = 0

        val ticks = 0.0

        while (opModeIsActive() && stopState <= 250) {
            val position = position
            val power = scoringPosition.power(ticks, position)

            telemetry.addData("stopstate: ", stopState)
            telemetry.addData("KP*error: ", scoringPosition.returnVal()[0])
            telemetry.addData("KI*i: ", scoringPosition.returnVal()[1])
            telemetry.addData("KD*d: ", scoringPosition.returnVal()[2])
            telemetry.update()

            hardware.pivot1.power = power
            hardware.pivot2.power = power

            if (Math.abs(ticks - position) <= Constants.ENCODER_TOLERANCE) {
                stopState = (System.nanoTime() - startTime) / 1000000
            } else {
                startTime = System.nanoTime()
            }
        }
        stop()

    }

    fun eReset() {
        for (motor in hardware.pivotMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        }
    }

    fun stop() {
        for (motor in hardware.pivotMotors) {
            motor.power = 0.0
        }
    }

    fun setLiftPosition(liftPosition: Double) {
        this.liftPosition = liftPosition
    }

    fun setPower(power: Double) {
        for (motor in hardware.pivotMotors) {
            motor.power = power
        }
    }

    fun opModeIsActive(): Boolean = auto.opModeIsActive


}
