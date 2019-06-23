package ftc.library.MaelSensors.MaelOdometry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelMotions.MaelMotors.MaelMotor;
import ftc.library.MaelMotions.MaelMotors.Motor;
import ftc.library.MaelRobot;
import ftc.library.MaelSensors.MaelIMU;
import ftc.library.MaelSubsystems.MaelstromDrivetrain.MaelDrivetrain;
import ftc.library.MaelUtils.MaelUtils;
import ftc.library.MaelUtils.LibConstants;

/*Class for odometry wheel tracking*/
public class MecanumOdometry extends Odometry implements LibConstants {


    public MecanumOdometry(MaelMotor x, MaelMotor y, MaelIMU imu, double gearRatio) {
        super(x, y, imu, gearRatio);
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
