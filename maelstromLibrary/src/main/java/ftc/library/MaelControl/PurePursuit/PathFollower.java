package ftc.library.MaelControl.PurePursuit;

import java.util.ArrayList;

import ftc.library.MaelRobot;
import ftc.library.MaelSensors.MaelOdometry.TankOdometry;
import ftc.library.MaelUtils.LibConstants;
import ftc.library.MaelUtils.MaelMath;
import ftc.library.MaelUtils.MaelUtils;
import ftc.library.MaelWrappers.MaelTellemetry;

public class PathFollower implements LibConstants {
    TankOdometry tracker;
    MaelRobot robot;
    MaelTellemetry feed;
    private MaelPose initial = new MaelPose(0,0);
    private MaelPose goal = new MaelPose(0,0);
    private double lookAhead = 1;
    private double distanceBetweenWheels = 18;
    double leftVelocity = 0;
    double rightVelocity = 0;
    private double r = 0;
    private ArrayList<MaelPose> wayPoints;

    public PathFollower(MaelRobot robot){
        this.robot = robot;
        tracker = robot.tankTracker;
        feed = MaelUtils.feed;
    }

    public MaelPose trackPosition(){
        return tracker.toVehiclePose(goal);
    }

    public double getDistance(){
/*        double current = tracker.getDistance();
        double goalDistance = Math.hypot(goal.x,goal.y);
        double error = goalDistance - current;
        return error;*/

        return MaelMath.calculateDistance(goal,tracker.toPose());
    }

    public double[] getWheelVelocities(){
        double omega = Math.toRadians(robot.imu.getYawVelocity()) * getRadius();
        double targetVelocity = omega/getCurvature();

        double left = targetVelocity * (2 + (getCurvature()*distanceBetweenWheels))/2;
        double right = targetVelocity * (2 - (getCurvature()*distanceBetweenWheels))/2;

        double []wheelVelocities = {left,right};

        return wheelVelocities;
    }

    public double getCurvature(){
        //double distanceError = (Math.exp(goal.x - initial.x) + Math.exp(goal.y));
        double curvature = (2*(goal.x - initial.x))/Math.exp(getDistance());
        return curvature;
    }

    public double getVelocity(){
        return robot.imu.getYawVelocity() * getCurvature();
    }

    public double[] deriveSpeeds(){
        double r = getRadius();
        double d = distanceBetweenWheels;
        double constantVelocity = (r - (d / 2))/(r + (d / 2));
        leftVelocity = rightVelocity * constantVelocity;
        double rightVelocity = leftVelocity * constantVelocity;
        double speeds[] = {leftVelocity,rightVelocity};
        return speeds;
    }

    public CurvePoint getFollowPoint(ArrayList<CurvePoint> pathPoints, MaelPose robot, double followradius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for(int i = 0; i < pathPoints.size() - 1; i++){
            CurvePoint startLine  = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<MaelPose> intersections = MaelMath.lineCircleIntersection(robot,followradius,startLine.toPose(),
                    endLine.toPose());

            double closestAngle = 1000000;

            for(MaelPose thisIntersection : intersections){
                double angle = Math.atan2(thisIntersection.y - tracker.getY(),thisIntersection.x - tracker.getX());
                double deltaAngle = Math.abs(MaelMath.anglewrap(angle - tracker.getHeading()));

                if(deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPose(thisIntersection);
                }
            }
        }
        return followMe;
    }

    public void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
        CurvePoint followMe = getFollowPoint(allPoints,tracker.toPose(),
                allPoints.get(0).followDistance);

        goToPostition(followMe.x,followMe.y,followMe.moveSpeed);

    }





    public void goToPostition(double x, double y, double speed){

        double robotX = tracker.getX();
        double robotY = tracker.getY();
        double robotAngle = tracker.getHeading();

        double distanceToTarget = Math.hypot(x - robotX,y - robotX);

        double absoluteAngleToTarget = Math.atan2(y - robotY,x - robotY);

        double relativeAngleToPoint = MaelMath.anglewrap(absoluteAngleToTarget - (robotAngle- Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementX = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementY = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        drive(movementX,movementY,0);

        feed.add("Distance to Target:", distanceToTarget);
        feed.add("Absolute Angle to Target:", absoluteAngleToTarget);
        feed.add("Relative X to Point:", relativeXToPoint);
        feed.add("Relative Y to Point:", relativeYToPoint);
        feed.add("X Movement:", movementX);
        feed.add("Y Movement:", movementY);
        feed.update();

    }

    public void drive(double xMovement, double yMovement, double turnMovement){
        double tl = yMovement + xMovement - turnMovement*1.5;
        double bl = yMovement - xMovement + turnMovement*1.5;
        double br = yMovement - xMovement + turnMovement*1.5;
        double tr = yMovement + xMovement - turnMovement*1.5;

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

        robot.dt.fl.setPower(tl);
        robot.dt.bl.setPower(bl);
        robot.dt.br.setPower(br);
        robot.dt.fr.setPower(tr);
    }


    public double getRadius(){
        double r = 1 / getCurvature();
        return r;
    }

 /*   public MaelPose getLookAheadPoint(){
        MaelVector start = wayPoints.get(0).toVector();
        MaelVector center = tracker.toVector();
        MaelVector end = wayPoints.get(wayPoints.size() - 1).toVector();

        MaelVector d = end.displacement(start);
        MaelVector f = start.displacement(center);

        double a = d.dotProdcut(d);
        double b = 2 * f.dotProdcut(d);
        double c = f.dotProdcut(f) - (lookAhead * lookAhead);
        double discriminant = (b*b) - 4*(a*c);

        if(discriminant < 0){
            //no intersection
        }
        else{
            discriminant = Math.sqrt(discriminant);
            double t1 = (-b - discriminant)/ (2*a);
            double t2 = (-b + discriminant) / (2*a);

            if (t1 >= 0 && t1 <= 1) {
                return t1;
            }
            }
        }

    }*/

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

    public MaelPose getGoal(){return goal;}

    public void setGoal(MaelPose goal){this.goal = goal;}

    public double getLookAhead(){return lookAhead;}

    public void setLookAhead(double lookAhead){this.lookAhead = lookAhead;}
}
