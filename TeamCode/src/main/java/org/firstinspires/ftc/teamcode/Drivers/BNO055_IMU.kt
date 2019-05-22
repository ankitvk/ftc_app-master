package org.firstinspires.ftc.teamcode.Drivers

import com.qualcomm.hardware.bosch.BNO055IMU

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode
import org.firstinspires.ftc.teamcode.Hardware.Hardware

import java.lang.Thread.sleep

class BNO055_IMU (name: String, hardware: Hardware): Runnable {

    private var relativeYaw = 0.0
    private var lastAngle = 0.0
    private val imu: BNO055IMU
    private val auto: AutonomousOpMode
    private var zeroPos = 0.0

    // for the Adafruit IMU, yaw and roll are switched
    val angles: DoubleArray
        get() {
            val quatAngles = imu.quaternionOrientation

            val w = quatAngles.w.toDouble()
            val x = quatAngles.x.toDouble()
            val y = quatAngles.y.toDouble()
            val z = quatAngles.z.toDouble()
            val roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 180.0 / Math.PI
            val pitch = Math.asin(2 * (w * y - x * z)) * 180.0 / Math.PI
            val yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 180.0 / Math.PI

            return doubleArrayOf(yaw, pitch, roll)
        }

    val yaw: Double
        get() = angles[0] - zeroPos

    val pitch: Double
        get() = angles[1]

    val roll: Double
        get() = angles[2]

    val adafruitYaw: Double
        get() = angles[2]

    init {
        imu = hardware.hwMap!!.get(BNO055IMU::class.java, name)
        setParameters()
        auto = hardware.auto!!
        val updateYaw =  Thread(this)
        updateYaw.start()
    }

    private fun setParameters() {
        val parameters = BNO055IMU.Parameters()
        parameters.mode = BNO055IMU.SensorMode.IMU
        parameters.useExternalCrystal = true
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        parameters.pitchMode = BNO055IMU.PitchMode.WINDOWS


        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"
        /*
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

        try{
            sleep(100); //Changing modes requires a delay before doing anything else
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);

        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);

        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        try{
            sleep(100); //Changing modes requires a delay before doing anything else
        } catch (InterruptedException e) {
            e.printStackTrace();
        }*/

        imu.initialize(parameters)
    }

    private fun updateRelativeYaw() {
        if (lastAngle > 90 && angles[0] < 0) {
            relativeYaw = 180 * Math.round(relativeYaw / 180) + (180 + yaw)
        } else if (lastAngle < -90 && yaw > 0) {
            relativeYaw = 180 * Math.round(relativeYaw / 180) - (180 - yaw)
        } else if (Math.abs(relativeYaw) <= 180) {
            relativeYaw = yaw
        } else {
            relativeYaw += yaw - lastAngle
        }
        lastAngle = yaw
    }

    fun getRelativeYaw() = relativeYaw

    fun resetYaw() {
        relativeYaw = 0.0
    }

    override fun run(){
        if(auto == null){
            return
        }
        while(!auto.opModeIsActive){}
        try{
            sleep(50)
        }
        catch (e: InterruptedException) {
            e.printStackTrace()
        }
        while(auto.opModeIsActive){
        updateRelativeYaw()
        try{
            sleep(10)}
        catch(ex : InterruptedException){
            ex.printStackTrace()
        }}
    }
}
