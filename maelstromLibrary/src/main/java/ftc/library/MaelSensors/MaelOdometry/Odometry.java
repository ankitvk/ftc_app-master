package ftc.library.MaelSensors.MaelOdometry;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelMotions.MaelMotors.MaelMotor;
import ftc.library.MaelSensors.MaelIMU;
import ftc.library.MaelSubsystems.Subsystem;

public abstract class Odometry implements Subsystem {
    MaelMotor x, y;
    MaelIMU imu;
    double globalX = 0, globalY = 0, prevX = 0, prevY = 0;
    double globalHeading = 0;
    double gearRatio = 1;
    public Odometry(MaelMotor x, MaelMotor y, MaelIMU imu, double gearRatio){
        this.x = x;
        this.y = y;
        this.imu = imu;
        this.gearRatio = gearRatio;
    }

    public Odometry(MaelMotor x, MaelMotor y, MaelIMU imu){
        this(x,y,imu,1);
    }

    @Override
    public void update(){
        updateTracker();
    }


    abstract void updateTracker();
    abstract double getX();
    abstract double getY();
    abstract double getHeading();

    public double getAngle(){
        return Math.hypot(getX(),getY());
    }

    public double getXCounts(){
        return x.getCounts();
    }

    public double getYCounts(){
        return y.getCounts();
    }

    public double calculateCircumference(double countsPerRev){
        return (2 * Math.PI) / (countsPerRev * gearRatio);
    }

    public double getDistance(){
        return Math.hypot(getX(),getY());
    }

    public MaelPose toPose(){
        return new MaelPose(getX(),getY(),getHeading());
    }

    public MaelPose toVehiclePose(MaelPose goal){
        double goalPointX = (goal.x - toPose().x)*Math.cos(getHeading()) + (goal.y - toPose().y)*Math.sin(getHeading());
        double goalPointY = (goal.x - toPose().x)*Math.sin(getHeading()) + (goal.y - toPose().y)*Math.cos(getHeading());
        return new MaelPose(goalPointX,goalPointY,getHeading());
    }


    public void reset(){
        x.stopAndReset();
        y.stopAndReset();
    }



}
