package ftc.library.MaelSensors.MaelOdometry;

import ftc.library.MaelMotions.MaelMotors.MaelMotor;
import ftc.library.MaelSensors.MaelIMU;
import ftc.library.MaelUtils.LibConstants;

public class MechOdometry_v2 extends Odometry implements LibConstants {

    private double globalX = 0, globalY = 0, prevX = 0, prevY = 0;
    private double globalHeading = 0;
    public MechOdometry_v2(MaelMotor x, MaelMotor y, MaelIMU imu) {
        super(x, y, imu);
    }

    @Override
    void updateTracker() {
        double xDelta = (getXCounts() - prevX);
        double yDelta = (getYCounts() - prevY);
        double theta = Math.toRadians(imu.getYaw());
        double x = xDelta * Math.cos(theta) - yDelta * Math.sin(theta);
        double y = xDelta * Math.sin(theta) + yDelta * Math.cos(theta);
        globalX += x;
        globalY += y;
        globalHeading = theta;
        prevX = getXCounts();
        prevY = getYCounts();
    }

    @Override
    double getX() {
        return globalX * calculateCircumference(x.getCPR());
    }

    @Override
    double getY() {
        return globalY * calculateCircumference(y.getCPR());
    }

    @Override
    double getHeading() {
        return globalHeading;
    }
}
