package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.PointF;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Drivers.BNO055_IMU;

public class PathFollower implements Constants {

    private AutonomousOpMode auto;
    private PointF coordinate  = new PointF(0,0);
    private Hardware robot;
    private BNO055_IMU imu;
    private Telemetry telemetry;

    private double R;

    public PathFollower(AutonomousOpMode auto, Hardware hardware, Telemetry telemetry){

        this.auto = auto;
        this.robot = hardware;
        this.imu = hardware.imu;
        this.telemetry = telemetry;
    }

    public PathFollower(Hardware hardware){
        this.robot = hardware;
        this.imu = hardware.imu;
    }

    private double[] follow(double desiredX, double desiredY, double lastX, double lastY, double currentX, double currentY){

        double Ax = desiredX - lastX;
        double Ay = desiredY - lastY;

        double Bx = currentX - lastX;
        double By = currentY - lastY;

        double legLength = Math.hypot(Ax,Ay);

        double proj = ((Ax*Bx)+(Ay*By))/legLength;

        double Rx = (proj + LOOKAHEAD)*Ax/legLength+lastX;
        double Ry = (proj + LOOKAHEAD)*Ay/legLength+lastY;

        double D = Math.hypot(Rx-currentX,Ry-currentY);
        R = D/(2*Math.sin(Math.toRadians(imu.getRelativeYaw())));
        double V = SPEED_MULTIPLIER;

        double[] speeds = new double[2];
        double velocity_right = (V*((2*R)+LENGTH_BETWEEN_WHEELS))/(2*R);
        double velocity_left = (2*V)-velocity_right;

        speeds[0] = velocity_left;
        speeds[1] = velocity_right;

        return speeds;
    }

    private PointF trackPosition (PointF coordinate, double oldEncoderTicks){
        double theta = Math.toRadians(imu.getRelativeYaw());
        double newEncoderTicks = (robot.frontLeft.getCurrentPosition()-robot.frontRight.getCurrentPosition())/2;
        double encoderTicks = newEncoderTicks-oldEncoderTicks;
        double magnitude = -encoderTicks*WHEEL_DIAMETER*Math.PI/DT_GEARBOX_TICKS_PER_ROTATION;
        float deltaX = (float)(magnitude*Math.cos(theta));
        float deltaY = (float)(magnitude*Math.sin(theta));
        coordinate.set(coordinate.x +deltaX,coordinate.y + deltaY);

        return coordinate;
    }

    private boolean OpModeState(){
        return auto.getOpModeIsActive();
    }

    public void trackPoint (double desiredX, double desiredY ){
        double speeds[];

        double lastX = coordinate.x;
        double lastY = coordinate.y;
        double currentX;
        double currentY;
        double oldEncoderTicks = (robot.frontLeft.getCurrentPosition()-robot.frontRight.getCurrentPosition())/2;

        while ((Math.hypot(desiredX-coordinate.x,desiredY-coordinate.y)>=0.5)&&(OpModeState())){

            PointF currPoint = trackPosition(coordinate,oldEncoderTicks);
            currentX = currPoint.x;
            currentY = currPoint.y;

            speeds = follow(desiredX,desiredY,lastX,lastY,coordinate.x,currentY);

            robot.frontLeft.setPower(-speeds[0]*PATH_FOLLOW_SPEED_MULTIPLIER);
            robot.backLeft.setPower(-speeds[0]*PATH_FOLLOW_SPEED_MULTIPLIER);
            robot.frontRight.setPower(speeds[1]*PATH_FOLLOW_SPEED_MULTIPLIER);
            robot.backRight.setPower(speeds[1]*PATH_FOLLOW_SPEED_MULTIPLIER);

            telemetry.addData("Left: ",speeds[0]*PATH_FOLLOW_SPEED_MULTIPLIER);
            telemetry.addData("Right: ",speeds[1]*PATH_FOLLOW_SPEED_MULTIPLIER );
            telemetry.addLine("");
            telemetry.addData("X-coordinate: ",coordinate.x);
            telemetry.addData("Y-coordinate: ",coordinate.y);
            telemetry.addData("Radius: ",R);
            telemetry.update();

            lastX = currentX;
            lastY = currentY;
            oldEncoderTicks = (robot.frontLeft.getCurrentPosition()-robot.frontRight.getCurrentPosition())/2;

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