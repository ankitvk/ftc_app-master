package ftc.library.MaelSensors.MaelOdometry;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelMotions.MaelMotors.MaelMotor;
import ftc.library.MaelSensors.MaelIMU;
import ftc.library.MaelSubsystems.MaelstromDrivetrain.MaelDrivetrain;
import ftc.library.MaelUtils.LibConstants;

public class TankOdometry implements LibConstants {
    private MaelDrivetrain dt;
    private MaelMotor left, right;
    public MaelIMU imu;
    private double previousPos = 0;
    private double wheelDiamater = 4;
    public double gearRatio;
    private double x = 0;
    private double y = 0;
    private double distance = 0;

    public TankOdometry(MaelDrivetrain dt, MaelIMU imu){
        this.dt = dt;
        this.imu = imu;
        left = dt.leftDrive.motor1;
        right = dt.rightDrive.motor1;
        gearRatio = dt.getDrivenGearReduction();
    }

    public MaelPose toPose(){
        return new MaelPose(trackX(),trackY(),trackAngle());
    }

    public double trackDistance(){
        distance = (trackLeft() + trackRight()) / 2;
        return getDistance(distance);
    }

    public double trackX(){
        double theta = radians(trackAngle());
        x += trackDistance()*cosine(theta);
        return x;
    }

    public double trackY(){
        double theta = radians(trackAngle());
        y += trackDistance()*sine(theta);
        return y;
    }

    public double trackAngle(){
        double theta = radians(imu.getYaw());
        return theta;
    }

    private double trackLeft(){
        double currPos = left.getCounts();
        double deltaPos = currPos - previousPos;
        previousPos = currPos;
        return deltaPos;
    }

    private double trackRight(){
        double currPos = right.getCounts();
        double deltaPos = currPos - previousPos;
        previousPos = currPos;
        return deltaPos;
    }

    public void reset(){
        dt.eReset();
        imu.resetAngle();
    }

    private double getDistance(double d){
        distance = (d*(wheelDiamater*Math.PI))/gearRatio;
        return d;
    }

    private double cosine(double c){
        return Math.cos(c);
    }
    private double sine(double s){
        return Math.sin(s);
    }
    private double radians(double r){return Math.toRadians(r);}

}
