package ftc.library.MaelSensors.MaelOdometry;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelMotions.MaelMotors.MaelMotor;
import ftc.library.MaelSensors.MaelIMU;
import ftc.library.MaelSubsystems.MaelstromDrivetrain.MaelDrivetrain;
import ftc.library.MaelUtils.LibConstants;

public class TankOdometry extends Odometry implements LibConstants {
    private MaelDrivetrain dt;
    private MaelMotor left, right;
    public MaelIMU imu;
    private double previousPos = 0;
    private double wheelDiamater = 4;
    public double gearRatio = 1;
    private double x = 0;
    private double y = 0;
    private double distance = 0;

    public TankOdometry(MaelDrivetrain dt, MaelIMU imu){
        super(dt.fl,dt.fr,imu,dt.getDrivenGearReduction());
        left = dt.fl;
        right = dt.fr;
        this.imu = imu;
        gearRatio = dt.getDrivenGearReduction();
    }

    @Override
    void updateTracker() {
        double leftDelta = (getXCounts() - prevX);
        double rightDelta = (getYCounts() - prevY);
        double distance = (leftDelta + rightDelta)/2;
        double theta = Math.toRadians(imu.getYaw());
        double x = distance * Math.cos(theta);
        double y = distance * Math.sin(theta);
        globalX += x;
        globalY += y;
        globalHeading = theta;
        prevX = getXCounts();
        prevY = getYCounts();
    }

    @Override
    public double getX() {
        return (globalX * calculateCircumference(left.getCPR()));
    }

    @Override
    public double getY() {
        return (globalY * calculateCircumference(right.getCPR()));
    }

    @Override
    public double getHeading() {
        return globalHeading;
    }
/*

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
*/

}
