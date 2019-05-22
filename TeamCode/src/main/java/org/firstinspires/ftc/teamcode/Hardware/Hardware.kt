package org.firstinspires.ftc.teamcode.Hardware

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxNackException
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode
import org.firstinspires.ftc.teamcode.Control.Constants
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor
import org.firstinspires.ftc.teamcode.Drivers.BNO055_IMU
import org.firstinspires.ftc.teamcode.Drivers.LEDRiver
import org.firstinspires.ftc.teamcode.Subsystems.Components.Drivetrain
import org.firstinspires.ftc.teamcode.Subsystems.Components.Endgame
import org.firstinspires.ftc.teamcode.Subsystems.Components.Extendo
import org.firstinspires.ftc.teamcode.Subsystems.Components.Hang
import org.firstinspires.ftc.teamcode.Subsystems.Components.Intake
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Crater.craterLeft
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Crater.craterMiddle
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Crater.craterRight
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Depot.depotLeft
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Depot.depotMiddle
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Depot.depotRight
import org.firstinspires.ftc.teamcode.Subsystems.Components.Pivot
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Prototype.KetoAuto
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Prototype.MineralTime

class Hardware : Constants {

    var hwMap: HardwareMap? = null
        internal set

    var auto: AutonomousOpMode? = null

    var telemetry: Telemetry? = null

    internal var revHub: LynxModule? = null

    var ledRiver: LEDRiver? = null

    var index: Servo? = null

    var hangLeftTop: CRServo? = null
    var hangLeftBottom: CRServo? = null
    var hangRightTop: CRServo? = null
    var hangRightBottom: CRServo? = null

    var hangLeftRelease: Servo? = null
    var hangRightRelease: Servo? = null

    var imu: BNO055_IMU? = null

    var limit: DigitalChannel? = null


    var frontLeft = SpeedControlledMotor(.0031, 0.0, 0.0, 1.0)
    var backLeft = SpeedControlledMotor(.0031, 0.0, 0.0, 1.0)
    var frontRight = SpeedControlledMotor(.0031, 0.0, 0.0, 1.0)
    var backRight = SpeedControlledMotor(.0031, 0.0, 0.0, 1.0)
    var extend = SpeedControlledMotor(Constants.Companion.extensionKP, Constants.Companion.extensionKI, Constants.Companion.extensionKD, Constants.Companion.extensionMaxI)
    var winch = SpeedControlledMotor(0.0, 0.0, 0.0, 1.0)
    var pivot1 = SpeedControlledMotor(0.0, 0.0, 0.0, 0.0)
    var pivot2 = SpeedControlledMotor(0.0, 0.0, 0.0, 0.0)

    var drivetrainMotors = arrayOf(frontLeft, backLeft, frontRight, backRight)

    var pivotMotors = arrayOf(pivot1, pivot2)

    var theHangGang = arrayOf(hangLeftTop, hangLeftBottom, hangRightTop, hangRightBottom)


    var drivetrain: Drivetrain? = null
    var pivot: Pivot? = null
    var extendo: Extendo? = null
    var endgame: Endgame? = null
    var hang: Hang? = null
    var intake: Intake? = null

    var depotLeft: depotLeft? = null
    var depotMiddle: depotMiddle? = null
    var depotRight: depotRight? = null

    var craterLeft: craterLeft? = null
    var craterMiddle: craterMiddle? = null
    var craterRight: craterRight? = null

    var ketoAuto: KetoAuto? = null
    var mineralTime: MineralTime? = null

    fun init(hardwareMap: HardwareMap) {

        this.hwMap = hardwareMap

        /*revHub = hwMap.get(LynxModule.class, "Expansion Hub 2");
        try {
            new LynxI2cConfigureChannelCommand(revHub, 1, LynxI2cConfigureChannelCommand.SpeedCode.FAST_400K).send();
        } catch (LynxNackException | InterruptedException ex) {
            ex.printStackTrace();
        }

        ledRiver = hardwareMap.get(LEDRiver.IMPL, "ledriver");

        ledRiver.setLEDCount(ledCount);
        ledRiver.setMode(LEDRiver.Mode.PATTERN);
        ledRiver.setLEDMode(LEDRiver.LEDMode.RGB);
        ledRiver.setColorDepth(LEDRiver.ColorDepth.BIT_16);*/

        imu = BNO055_IMU("imu", this)


        frontLeft.init(hwMap!!, "frontLeft")
        frontRight.init(hwMap!!, "frontRight")
        backLeft.init(hwMap!!, "backLeft")
        backRight.init(hwMap!!, "backRight")

        extend.init(hwMap!!, "extendo")

        pivot1.init(hwMap!!, "pivot1")
        pivot2.init(hwMap!!, "pivot2")

        winch.init(hwMap!!, "winch")

        index = hardwareMap.servo.get("index")

        hangLeftBottom = hardwareMap.crservo.get("hangLeftBottom")
        hangLeftTop = hardwareMap.crservo.get("hangLeftTop")
        hangRightBottom = hardwareMap.crservo.get("hangRightBottom")
        hangRightTop = hardwareMap.crservo.get("hangRightTop")

        hangLeftRelease = hardwareMap.servo.get("hangLeftRelease")
        hangRightRelease = hardwareMap.servo.get("hangRightRelease")

        drivetrain = Drivetrain(this)
        pivot = Pivot(this)
        extendo = Extendo(this)
        endgame = Endgame(this)
        hang = Hang(this)
        intake = Intake(this)

        depotLeft = depotLeft(this)
        depotMiddle = depotMiddle(this)
        depotRight = depotRight(this)

        craterLeft = craterLeft(this)
        craterMiddle = craterMiddle(this)
        craterRight = craterRight(this)

        ketoAuto = KetoAuto(this)
        mineralTime = MineralTime(this)

        limit = hardwareMap.digitalChannel.get("limit")

        extendo!!.setTargetPosition()

    }

    fun opportunityInit(hardwareMap: HardwareMap) {
        this.hwMap = hardwareMap

        frontLeft.init(hwMap!!, "frontLeft")
        frontRight.init(hwMap!!, "frontRight")
        backLeft.init(hwMap!!, "backLeft")
        backRight.init(hwMap!!, "backRight")

        imu = BNO055_IMU("imu", this)

        drivetrain = Drivetrain(this)
    }

    fun setAuto(auto: AutonomousOpMode, telemetry: Telemetry) {
        this.auto = auto
        this.telemetry = telemetry
    }

}