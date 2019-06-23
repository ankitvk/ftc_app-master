package ftc.library.MaelControl.PurePursuit;

import ftc.library.MaelRobot;
import ftc.library.MaelSensors.MaelOdometry.TankOdometry;
import ftc.library.MaelUtils.LibConstants;
import ftc.library.MaelUtils.MaelUtils;
import ftc.library.MaelWrappers.MaelTellemetry;

public class PathFollower implements LibConstants {
    TankOdometry tracker;
    MaelRobot robot;
    MaelTellemetry feed;
    MaelPose initial = new MaelPose(0,0);

    public PathFollower(MaelRobot robot){
        this.robot = robot;
        tracker = robot.tankTracker;
        feed = MaelUtils.feed;
    }

    public MaelPose trackPosition(MaelPose goal){
        return tracker.toVehiclePose(goal);
    }

    public double getDistanceError(MaelPose goal){
        double current = tracker.getDistance();
        double goalDistance = Math.hypot(goal.x,goal.y);
        double error = goalDistance - current;
        return error;
    }

    public double getCurvature(MaelPose goal){
        //double distanceError = (Math.exp(goal.x - initial.x) + Math.exp(goal.y));
        double curvature = (2*(goal.x - initial.x))/Math.exp(getDistanceError(goal));
        return curvature;
    }

    public static double[][] doubleArrayCopy(double[][] arr)
    {

        //size first dimension of array
        double[][] temp = new double[arr.length][arr[0].length];

        for(int i=0; i<arr.length; i++)
        {
            //Resize second dimension of array
            temp[i] = new double[arr[i].length];

            //Copy Contents
            for(int j=0; j<arr[i].length; j++)
                temp[i][j] = arr[i][j];
        }

        return temp;

    }

    public double[][] smoother(double[][] path, double weight_data, double weight_smooth, double tolerance)
    {

        //copy array
        double[][] newPath = doubleArrayCopy(path);

        double change = tolerance;
        while(change >= tolerance)
        {
            change = 0.0;
            for(int i=1; i<path.length-1; i++)
                for(int j=0; j<path[i].length; j++)
                {
                    double aux = newPath[i][j];
                    newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
        }

        return newPath;

    }
}
