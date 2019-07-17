package ftc.library.MaelSensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import ftc.library.MaelSubsystems.Subsystem;
import ftc.library.MaelUtils.AxesSigns;
import ftc.library.MaelUtils.LibConstants;
import ftc.library.MaelUtils.MaelUtils;
import ftc.library.MaelWrappers.MaelLinearOp;

/*Custon class for gyro imu*/
public class MaelIMU implements LibConstants, Subsystem/*Runnable*/ {

    private double relativeYaw = 0;
    private double lastAngle = 0;
    private double previousPos = 0, previousTime = 0;
    private double yawVelocity = 0;
    private double yaw = 0;
    private double pitch = 0;
    private double roll = 0;
    private final BNO055IMU imu;
    private MaelLinearOp linearOp = MaelUtils.linearOpMode;

    public MaelIMU(String name, HardwareMap hwMap){
        imu = hwMap.get(BNO055IMU.class,name);
        setParameters();
        // auto = hardware.auto;
        /*Thread updateYaw = new Thread(this);
        updateYaw.start();*/
    }

    @Override
    public void update() throws InterruptedException {

        while(!linearOp.opModeIsActive()){
            updateAngles();
            updateRelativeYaw();
            try{Thread.sleep(10);}
            catch (InterruptedException e){ e.printStackTrace(); }
        }
        while(!linearOp.isStopRequested()){
            updateAngles();
            updateRelativeYaw();
            try{Thread.sleep(10);}
            catch (InterruptedException e){ e.printStackTrace(); }
        }
    }


    private void setParameters(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.useExternalCrystal = true;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.pitchMode = BNO055IMU.PitchMode.WINDOWS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
    }

    public void updateRelativeYaw(){
        if (lastAngle > 90 && getYaw() < 0) {
            relativeYaw = 180 * Math.round(relativeYaw/180) + (180 + getYaw() );
        }
        else if (lastAngle < -90 && getYaw()  > 0) {
            relativeYaw = 180 * Math.round(relativeYaw/180) - (180 - getYaw() );
        }
        else if (Math.abs(relativeYaw) <= 180) {
            relativeYaw = getYaw();
        }
        else {
            relativeYaw += getYaw()  - lastAngle;
        }
        lastAngle = getYaw();
    }

/*    public double getYaw(){
        updateAngles();
        return yaw;
    }*/

    public double getRelativeYaw(){
        //updateRelativeYaw();
        return relativeYaw;}

    public void updateAngles(){
        Quaternion quatAngles = imu.getQuaternionOrientation();

        double w = quatAngles.w;
        double x = quatAngles.x;
        double y = quatAngles.y;
        double z = quatAngles.z;

        roll = Math.atan2( 2*(w*x + y*z) , 1 - (2*(x*x + y*y)) ) * 180.0 / Math.PI;
        pitch = Math.asin( 2*(w*y - x*z) ) * 180.0 / Math.PI;
        yaw = Math.atan2( 2*(w*z + x*y), 1 - (2*(y*y + z*z)) ) * 180.0 / Math.PI;
    }

    public void resetAngle(){
        yaw = 0;
        relativeYaw = 0;
    }

    public double adjustAngle(double angle){
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    public double getYaw() {
        //updateAngles();
        return yaw;}
    public double getRoll(){return roll;}
    public double getPitch(){return pitch;}

    public double getYawVelocity(){
        double deltaPos = getYaw() - previousPos;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_MIN;
        if (deltaTime*6e4 > 10) {
            yawVelocity = deltaPos/deltaTime;
            previousPos = getYaw();
            previousTime = System.nanoTime();
        }
        return yawVelocity;
    }

    /*public void run(){
        if(auto == null){
            return;
        }
        while (!auto.getOpModeIsActive()) {
            try {
                Thread.sleep(50);
            }
            catch (InterruptedException e) {

            }
        }

        while (auto.getOpModeIsActive()) {
            updateRelativeYaw();
            try {
                Thread.sleep(10);
            }
            catch (InterruptedException e) {
            }
        }
    }*/



    public void write8(BNO055IMU.Register register, int bVal){
        imu.write8(register,bVal);
    }

    public BNO055IMU.Parameters getParameters(){
        return imu.getParameters();
    }

    public static void remapAxes(MaelIMU imu, AxesOrder order, AxesSigns signs) {
        try {
            // the indices correspond with the 2-bit encodings specified in the datasheet
            int[] indices = order.indices();
            int axisMapConfig = 0;
            axisMapConfig |= (indices[0] << 4);
            axisMapConfig |= (indices[1] << 2);
            axisMapConfig |= (indices[2] << 0);

            // the BNO055 driver flips the first orientation vector so we also flip here
            int axisMapSign = signs.bVal ^ (0b100 >> indices[0]);

            // Enter CONFIG mode
            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

            Thread.sleep(100);

            // Write the AXIS_MAP_CONFIG register
            imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, axisMapConfig & 0x3F);

            // Write the AXIS_MAP_SIGN register
            imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, axisMapSign & 0x07);

            // Switch back to the previous mode
            imu.write8(BNO055IMU.Register.OPR_MODE, imu.getParameters().mode.bVal & 0x0F);

            Thread.sleep(100);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public String dataOutput() {
        return String.format("Yaw: %.3f  Pitch: %.3f  Roll: %.3f", getYaw(), getPitch(), getRoll());
    }


}
