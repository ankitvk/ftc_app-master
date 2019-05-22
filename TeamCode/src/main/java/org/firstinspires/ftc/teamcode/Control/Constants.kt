package org.firstinspires.ftc.teamcode.Control

import com.qualcomm.hardware.rev.RevBlinkinLedDriver

interface Constants {
    companion object {
        val NANOSECONDS_PER_MINUTE = 6e+10
        val NEVEREST_CLASSIC_MAX_RPM = 6600.0
        val NEVEREST20_COUNTS_PER_REV = 560.0
        val NEVEREST40_COUNTS_PER_REV = 1120.0

        val NEVEREST_CLASSIC_TICKS_PER_ROTATION = 28.0
        val DT_GEAR_RATIO = (55 / 3).toDouble()
        val mecanum_ratio = 1.0
        val DT_GEARBOX_TICKS_PER_ROTATION = DT_GEAR_RATIO * NEVEREST_CLASSIC_TICKS_PER_ROTATION
        val DT_MAX_RPM = NEVEREST_CLASSIC_MAX_RPM / DT_GEAR_RATIO
        val PIVOT_GEAR_RATIO = 235.0
        val PIVOT_LIMIT_ANGLE = 48.1

        val PIVOT_TICKS_PER_ROTATION = PIVOT_GEAR_RATIO * NEVEREST_CLASSIC_TICKS_PER_ROTATION
        val PIVOT_TICKS_PER_INCH = 100.0

        val E4T_COUNTS_PER_REV = 537.6
        val PATH_FOLLOWING_INTERVAL = 50
        val SPEED_MULTIPLIER = .85
        val PATH_FOLLOW_SPEED_MULTIPLIER = .25

        val LENGTH_BETWEEN_WHEELS = 15.789
        val WHEEL_DIAMETER = 4.0

        val DISTANCE_TOLERANCE = 1.5
        val IMU_TOLERANCE = .5
        val ENCODER_TOLERANCE = 50.0

        val LOOKAHEAD = 5.0

        val ALIGN_POSITION = 55

        val TARGET_GOLD_X_POS = 425

        val ledCount = 150

        val MIN_ROTATE_POWER = 0.39
        val MIN_DRIVE_POWER = 0.0

        val dtKP = .000175
        val dtKI = 0.01/*.035*/
        val dtKD = 0.0
        val dtMaxI = 1.0

        val dtRotateKP = 0.012
        val dtRotateKI = 1.0
        val dtRotateKD = 0.0
        val dtRotateMaxI = 1.0

        val dtBigRotateKP = 0.01285
        val dtBigRotateKI = 2.0
        val dtBigRotateKD = 0.0
        val dtBigRotateMaxI = 1.0

        val extensionKP = .015
        val extensionKI = 0.0
        val extensionKD = 0.0
        val extensionMaxI = 1.0

        val pivotKP = 1e-6
        val pivotKI = 0.0
        val pivotKD = 0.0
        val pivotMaxI = 1.0

        //tarun's stuff
        val PIVOT_UP = 1.0
        val PIVOT_DOWN = -1.0

        val INTAKE_POWER = 1.0
        val OUTTAKE_POWER = -1.0

        val LIFT_EXTEND = 1.0
        val LIFT_RETRACT = -1.0

        val distanceKP = 0.00125
        val distanceKI = 0.1
        val distanceKD = 0.0
        val distanceMaxI = 1.0

        //angle correction PID
        val angleCorrectionKP = 0.001
        val angleCorrectionKI = 0.0
        val angleCorrectionKD = 0.0
        val angleCorrectionMaxI = 1.0

        //turn angle (< 50) PID
        val turnKP = 0.03
        val turnKI = 0.0
        val turnKD = 0.0
        val turnMaxI = 1.0

        //turn big angle (> 50) PID
        val turnBigKP = 0.039
        val turnBigKI = 0.0
        val turnBigKD = 0.0
        val turnBigMaxI = 1.0

        //test turn PID
        val testTurnKP = 0.002
        val testTurnKI = 0.0
        val testTurnKD = 0.0
        val testTurnMaxI = 1.0

        val bigTestTurnKP = 0.013
        val bigTestTurnKI = 0.0
        val bigTestTurnKD = 0.0
        val bigTestTurnMaxI = 1.0

        //turn one side PID
        val sideKP = 0.001
        val sideKI = 0.0
        val sideKD = 0.0
        val sideMaxI = 1.0

        val bigSideKP = 0.001
        val bigSideKI = 0.0
        val bigSideKD = 0.0
        val bigSideMaxI = 1.0

        val LICENSE_KEY = "AbxcZxf/////AAABmYehhkt9/k+lgWzWDZFG64eH2XR0TK1U2WbaJWSXrI" +
                "BuB4xBU9FpE1oHH3WHomRfTKwInQH9f18rQNXlH0LoYQmYHUR30N4" +
                "7eierR8KeOCKWOz/ew5qYXDq5W4hy20SDt6/GrORnhVttBPrp1lu+RllaRA" +
                "NDz4PKdioMn8uobSLb4MC9z77cXv+BCxSQGsswhgaVNYh/3GoBLrYYEYDpgGGFRL" +
                "EJtrmf8pYbd3s+KKatCS5xEaEoILRYWQZMt5kENV2CyoaSznfg+zmRNeNpVpFBLWMl" +
                "aVoT19fBzj2IZZx209ztGb+MLCRkHwy2b0cTDc1DfBm/RfzcJsAm8ym9sAPTmlbi6UlWB/dJr9s48kwM"

        val PATTERN: RevBlinkinLedDriver.BlinkinPattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE
    }


}
