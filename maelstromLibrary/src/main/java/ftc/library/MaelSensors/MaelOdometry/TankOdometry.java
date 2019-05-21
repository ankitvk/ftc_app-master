package ftc.library.MaelSensors.MaelOdometry;

import ftc.library.MaelMotions.MaelMotors.MaelMotor;
import ftc.library.MaelSensors.MaelIMU;
import ftc.library.MaelSubsystems.MaelstromDrivetrain.MaelDrivetrain;
import ftc.library.MaelUtils.LibConstants;

public class TankOdometry implements LibConstants {
    private MaelDrivetrain dt;
    private MaelMotor left, right;
    public MaelIMU imu;

    private double previousPos = 0;
    private long previousTime = 0;
    private double rpm = 0;
    private double power;
    private double target;
    private double wheelDiamater = 4;
    private double gearRatio;
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
        /*int deltaPos = motor.getCurrentPosition() - previousPos;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_MIN;
        if (deltaTime*6e4 > 10) {
            rpm = (deltaPos/ getCPR())/(deltaTime);
            previousPos = motor.getCurrentPosition();
            previousTime = System.nanoTime();
        }
        return rpm;*/

    public double trackDistance(){
        //double deltaPos = getCurrCounts() - previousPos;
        distance = getCurrCounts() - previousPos;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_SEC;
        if(deltaTime*6e4 > 10){
            //distance = (deltaPos*(wheelDiamater*Math.PI))/gearRatio;
            distance = getDistance(distance);
            previousPos = distance;
            previousTime = System.nanoTime();
        }
        return distance;
    }

    public double getDistance(double d){
        distance = (d*(wheelDiamater*Math.PI))/gearRatio;
        return d;
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

    public void reset(){
        dt.eReset();
        imu.resetAngle();
    }


    private double getLeftCounts(){
        return left.getCounts();
    }

    private double getRightCounts(){
        return right.getCounts();
    }

    private double getCurrCounts(){
        return (getLeftCounts() + getRightCounts()) / 2;
    }

    private double cosine(double c){
        return Math.cos(c);
    }

    private double sine(double s){
        return Math.sin(s);
    }

    private double radians(double r){return Math.toRadians(r);}

}
