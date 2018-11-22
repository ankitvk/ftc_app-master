package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.PointF;

import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;

public class PathFollower implements Constants {

    private AutonomousOpMode auto;
    private PointF coordinate  = new PointF(0,0);

    public PathFollower(AutonomousOpMode auto){
        this.auto = auto;
    }

    public PathFollower(){}

    private double[] follow(BNO055_IMU imu, double desiredX, double desiredY, double lastX, double lastY, double currentX, double currentY){

        double Ax = desiredX - lastX;
        double Ay = desiredY - lastY;

        double Bx = currentX - lastX;
        double By = currentY - lastY;

        double legLength = Math.hypot(Ax,Ay);

        double proj = ((Ax*Bx)+(Ay*By))/legLength;

        double Rx = (proj + LOOKAHEAD)*Ax/legLength+lastX;
        double Ry = (proj + LOOKAHEAD)*Ay/legLength+lastY;

        double D = Math.hypot(Rx-currentX,Ry-currentY);
        double R = D/(2*Math.sin(imu.getYaw()));
        double V = SPEED_MULTIPLIER;

        double[] speeds = new double[2];
        double velocity_right = (V*((2*R)+LENGTH_BETWEEN_WHEELS))/(2*R);
        double velocity_left = (2*V)-velocity_right;

        speeds[0] = velocity_left;
        speeds[1] = velocity_right;

        return speeds;
    }

    private PointF trackPosition (BNO055_IMU imu, PointF coordinate, Hardware robot, double oldEncoderTicks){
        double theta = imu.getYaw();
        double newEncoderTicks = (robot.backRight.getCurrentPosition()+robot.frontRight.getCurrentPosition())/2;
        double encoderTicks = newEncoderTicks-oldEncoderTicks;
        double magnitude = encoderTicks*WHEEL_DIAMETER*Math.PI/NEVEREST20_COUNTS_PER_REV;
        float deltaX = (float)(magnitude*Math.cos(theta));
        float deltaY = (float)(magnitude*Math.sin(theta));
        coordinate.set(coordinate.x +deltaX,coordinate.y + deltaY);

        return coordinate;
    }

    private boolean OpModeState(){
        return auto.getOpModeIsActive();
    }

    public void trackPoint (Hardware robot, BNO055_IMU imu, double desiredX, double desiredY ){
        double speeds[];

        double lastX = coordinate.x;
        double lastY = coordinate.y;
        double currentX;
        double currentY;
        double oldEncoderTicks = (robot.backRight.getCurrentPosition()+robot.frontRight.getCurrentPosition())/2;

        while ((Math.hypot(desiredX-coordinate.x,desiredY-coordinate.y)>=0.5)&&(OpModeState())){

            PointF currPoint = trackPosition(imu,coordinate,robot,oldEncoderTicks);
            currentX = currPoint.x;
            currentY = currPoint.y;

            speeds = follow(imu,desiredX,desiredY,lastX,lastY,coordinate.x,currentY);

            robot.frontLeft.setPower(speeds[0]*SPEED_MULTIPLIER);
            robot.backLeft.setPower(speeds[0]*SPEED_MULTIPLIER);
            robot.frontRight.setPower(-speeds[1]*SPEED_MULTIPLIER);
            robot.backRight.setPower(-speeds[1]*SPEED_MULTIPLIER);

            lastX = currentX;
            lastY = currentY;
            oldEncoderTicks = (robot.backRight.getCurrentPosition()+robot.frontRight.getCurrentPosition())/2;

            try {
                Thread.sleep(PATH_FOLLOWING_INTERVAL);
            }
            catch(InterruptedException e) {
                e.printStackTrace();
            }

            if(!OpModeState()){
                break;
            }

        }
    }
}