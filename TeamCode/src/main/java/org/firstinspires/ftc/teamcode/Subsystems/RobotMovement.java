package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drivers.BNO055_IMU;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Drivetrain;

import ftc.library.MaelSensors.MaelOdometry.TankOdometry;

public class RobotMovement {

    private double previousPos = 0;
    private double wheelDiamater = 4;
    public double gearRatio;
    private double x = 0;
    private double y = 0;
    private double distance = 0;

    public Drivetrain dt;
    public BNO055_IMU imu;
    public Telemetry telemetry;

    public RobotMovement(Hardware robot){
        this.dt = robot.drivetrain;
        this.imu = robot.imu;
        this.telemetry = robot.telemetry;
    }
    //13:56

    private double anglewrap(double angle){
        while(angle < -Math.PI){
            angle += 2 * Math.PI;
        }
        while(angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public void goToPosition(double x, double y, double speed){
        double distanceToTarget = Math.hypot(x - trackX(),y - trackX());

        double absoluteAngleToTarget = Math.atan2(y - trackY(),x - trackX());

        double relativeAngleToPoint = anglewrap(absoluteAngleToTarget - (trackAngle()- Math.toRadians(90)));

        double relativeXToPoint = cosine(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = sine(relativeAngleToPoint) * distanceToTarget;

        double movementX = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementY = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        drive(movementX * speed,movementY * speed,0);

        telemetry.addData("Distance to Target:", distanceToTarget);
        telemetry.addData("Absolute Angle to Target:", absoluteAngleToTarget);
        telemetry.addData("Relative X to Point:", relativeXToPoint);
        telemetry.addData("Relative Y to Point:", relativeYToPoint);
        telemetry.addData("Current X:", trackX());
        telemetry.addData("Current Y:", trackY());
        telemetry.addData("X Movement:", movementX);
        telemetry.addData("Y Movement:", movementY);
        telemetry.update();

    }

    public void drive(double xMovement, double yMovement, double turnMovement){
        double tl = yMovement + xMovement - turnMovement*1.5;
        double bl = yMovement - xMovement + turnMovement*1.5;
        double br = yMovement - xMovement + turnMovement*1.5;
        double tr = yMovement + xMovement - turnMovement*1.5;

        /*
        * BackRight      BackLeft
        *
        *
        * TopRight       TopLeft
        *
        * */

        double maxRawPower = Math.abs(tl);
        if(Math.abs(bl) > maxRawPower){ maxRawPower = Math.abs(bl);}
        if(Math.abs(br) > maxRawPower){ maxRawPower = Math.abs(br);}
        if(Math.abs(tr) > maxRawPower){ maxRawPower = Math.abs(tr);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        tl *= scaleDownAmount;
        bl *= scaleDownAmount;
        br *= scaleDownAmount;
        tr *= scaleDownAmount;

        dt.frontLeft.setPower(-tl);
        dt.backLeft.setPower(bl);
        dt.backRight.setPower(br);
        dt.frontRight.setPower(-tr);
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
        double currPos = (dt.backRight.getCurrentPosition() + dt.frontRight.getCurrentPosition()) / 2;
        double deltaPos = currPos - previousPos;
        previousPos = currPos;
        return deltaPos;
    }

    private double trackRight(){
        double currPos = (dt.backRight.getCurrentPosition() + dt.frontRight.getCurrentPosition()) / 2;
        double deltaPos = currPos - previousPos;
        previousPos = currPos;
        return deltaPos;
    }

    public void reset(){
        dt.eReset();
        imu.resetYaw();
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
