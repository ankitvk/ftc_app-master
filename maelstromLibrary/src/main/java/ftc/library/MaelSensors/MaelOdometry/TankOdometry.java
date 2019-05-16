package ftc.library.MaelSensors.MaelOdometry;

import ftc.library.MaelMotions.MaelMotors.MaelMotor;
import ftc.library.MaelSensors.MaelIMU;
import ftc.library.MaelSubsystems.MaelstromDrivetrain.MaelDrivetrain;

public class TankOdometry {
    private MaelDrivetrain dt;
    private MaelMotor left, right;
    public MaelIMU imu;

    private int previousPos = 0;
    private long previousTime = 0;
    private double rpm = 0;
    private double power;
    private double target;
    private double wheelDiamater = 4;
    private double gearRatio = 1;
    private double x = 0;
    private double y = 0;

    public TankOdometry(MaelDrivetrain dt, MaelIMU imu){
        this.dt = dt;
        this.imu = imu;
        left = dt.leftDrive.motor1;
        right = dt.rightDrive.motor1;
        gearRatio = dt.getDrivenGearReduction();
    }

    public double trackDistance(){
        double currCounts = getCurrCounts();
        double distance = (currCounts*(wheelDiamater*Math.PI))/gearRatio;
        return distance;
    }

    public double trackX(){
        x += trackDistance()*cosine(trackAngle());
        return x;
    }

    public double trackY(){
        y += trackDistance()*sine(trackAngle());
        return y;
    }

    public double trackAngle(){
        return imu.getYaw();
    }

    public void reset(){
        dt.eReset();
        imu.resetAngle();
    }

    public double getLeftCounts(){
        return left.getCounts();
    }

    public double getRightCounts(){
        return right.getCounts();
    }

    public double getCurrCounts(){
        return (getLeftCounts() + getRightCounts()) / 2;
    }

    private double cosine(double c){
        return Math.cos(c);
    }

    private double sine(double s){
        return Math.sin(s);
    }

}
