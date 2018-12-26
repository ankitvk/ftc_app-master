package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

import static java.lang.Thread.sleep;

public class BNO055_IMU /*implements Runnable*/ {

    private double relativeYaw = 0;
    private double lastAngle = 0;
    private final BNO055IMU imu;
    private AutonomousOpMode auto;
    private double zeroPos = 0;



    public BNO055_IMU(String name, Hardware hardware) {
        imu = hardware.getHwMap().get(BNO055IMU.class, name);
        setParameters();
        auto = hardware.auto;
        /*Thread updateYaw = new Thread(this);
        updateYaw.start();*/
        }

    private void setParameters() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.useExternalCrystal = true;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.pitchMode = BNO055IMU.PitchMode.WINDOWS;


        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

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
        }

        imu.initialize(parameters);
    }

    private void updateRelativeYaw() {
        if (lastAngle > 90 && getAngles()[0] < 0) {
            relativeYaw = 180 * Math.round(relativeYaw/180) + (180 + getYaw() );
        } else if (lastAngle < -90 && getYaw()  > 0) {
            relativeYaw = 180 * Math.round(relativeYaw/180) - (180 - getYaw() );
        } else if (Math.abs(relativeYaw) <= 180) {
            relativeYaw = getYaw();
        } else {
            relativeYaw += getYaw() - lastAngle;
        }
        lastAngle = getYaw();
    }

    public double getRelativeYaw() {
        updateRelativeYaw();
        return relativeYaw;
    }

    public double[] getAngles() {
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
        return getAngles()[0] - zeroPos;
    }

    public double getPitch() { return getAngles()[1]; }

    public double getRoll() {
        return getAngles()[2];
    }

    public double getAdafruitYaw()
    {
        return getAngles()[2];
    }

 /*   public void run() {

        if (auto == null) {
            return;
        }
        while (!auto.getOpModeIsActive()) {
            try {
                sleep(50);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        while (auto.getOpModeIsActive()) {
            updateRelativeYaw();
            try {
                sleep(10);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }*/

    public void reset(){
        zeroPos = getRelativeYaw();
    }

    public String dataOutput() {
        return String.format("Yaw: %.3f  Pitch: %.3f  Roll: %.3f", getAngles()[0], getAngles()[1], getAngles()[2]);
    }
}
