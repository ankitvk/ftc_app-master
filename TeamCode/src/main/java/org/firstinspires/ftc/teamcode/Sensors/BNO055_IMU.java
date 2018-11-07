package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class BNO055_IMU implements Runnable {

    private double relativeYaw = 0;
    private double lastAngle = 0;
    private final BNO055IMU imu;
    private AutonomousOpMode auto;


    public BNO055_IMU(String name, Hardware hardware) {
        imu = hardware.getHwMap().get(BNO055IMU.class, name);
        setParameters();
        auto = hardware.auto;
        Thread updateYaw = new Thread(this);
        updateYaw.start();
    }

    private void setParameters() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.useExternalCrystal = true;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.pitchMode = BNO055IMU.PitchMode.WINDOWS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
    }

    private void updateRelativeYaw() {
        if (lastAngle > 90 && getYaw() < 0) {
            relativeYaw = 180 * Math.round(relativeYaw/180) + (180 + getYaw() );
        } else if (lastAngle < -90 && getYaw()  > 0) {
            relativeYaw = 180 * Math.round(relativeYaw/180) - (180 - getYaw() );
        } else if (Math.abs(relativeYaw) <= 180) {
            relativeYaw = getYaw();
        } else {
            relativeYaw += getYaw()  - lastAngle;
        }
        lastAngle = getYaw();
    }

    public double getRelativeYaw() {
        return relativeYaw;
    }

    private double[] getAngles() {
        Quaternion quatAngles = imu.getQuaternionOrientation();

        double w = quatAngles.w;
        double x = quatAngles.x;
        double y = quatAngles.y;
        double z = quatAngles.z;

        // for the Adafruit IMU, yaw and roll are switched
        double roll = Math.atan2( 2*(w*x + y*z) , 1 - (2*(x*x + y*y)) ) * 180.0 / Math.PI;
        double pitch = Math.asin( 2*(w*y - x*z) ) * 180.0 / Math.PI;
        double yaw = Math.atan2( 2*(w*z + x*y), 1 - (2*(y*y + z*z)) ) * 180.0 / Math.PI;

        return new double[]{yaw, pitch, roll};
    }

    public double getYaw() {
        return getAngles()[0];
    }

    public double getAdafruitYaw()
    {
        return getAngles()[2];
    }

    public void run() {

        if (auto == null) {
            return;
        }
        while (!auto.getOpModeIsActive()) {
            try {
                Thread.sleep(50);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        while (auto.getOpModeIsActive()) {
            updateRelativeYaw();
            try {
                Thread.sleep(10);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public String dataOutput() {
        return String.format("Yaw: %.3f  Pitch: %.3f  Roll: %.3f", getAngles()[0], getAngles()[1], getAngles()[2]);
    }
}
