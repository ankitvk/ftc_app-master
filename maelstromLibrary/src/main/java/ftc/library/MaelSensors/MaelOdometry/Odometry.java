package ftc.library.MaelSensors.MaelOdometry;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelControl.PurePursuit.MaelVector;
import ftc.library.MaelMotions.MaelMotors.MaelMotor;
import ftc.library.MaelSensors.MaelIMU;
import ftc.library.MaelSubsystems.Subsystem;
import ftc.library.MaelUtils.MaelMath;

public abstract class Odometry implements Subsystem {
    MaelMotor x, y;
    MaelIMU imu;
    double globalX = 0, globalY = 0, prevX = 0, prevY = 0;
    double globalHeading = 0;
    double gearRatio = 1;
    double wheelDiamter = 4;
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
        return (wheelDiamter * Math.PI * gearRatio) / (countsPerRev);
    }

    public double getDistance(){
        //return Math.hypot(getX(),getY());
        return MaelMath.calculateDistance(new MaelPose(0,0),toPose());
    }

    public MaelPose toPose(){
        return new MaelPose(getX(),getY(),getHeading());
    }

    public MaelVector toVector(){return new MaelVector(getX(),getY());}

    public MaelPose toVehiclePose(MaelPose goal){
        double goalPointX = (goal.x - toPose().x)*Math.cos(getHeading()) + (goal.y - toPose().y)*Math.sin(getHeading());
        double goalPointY = (goal.x - toPose().x)*Math.sin(getHeading()) + (goal.y - toPose().y)*Math.cos(getHeading());
        return new MaelPose(goalPointX,goalPointY,getHeading());
    }

    public MaelPose centerPose(double distanceFromCenter, double phi){
        MaelPose encoderPose = toPose();
        //double phi = 0;
        double xCenter = encoderPose.x + distanceFromCenter*Math.cos(phi + imu.getYaw());
        double yCenter = encoderPose.y + distanceFromCenter*Math.sin(phi + imu.getYaw());
        return new MaelPose(xCenter,yCenter,imu.getYaw() );
    }

    public void reset(){
        x.stopAndReset();
        y.stopAndReset();
    }



}
