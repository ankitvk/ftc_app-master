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
    private MaelPose current;
    private MaelPose goal = new MaelPose(0,0);
    private double lookAhead = 1;
    public double distanceBetweenWheels = 18;
    double leftVelocity = 0;
    double rightVelocity = 0;
    private double r = 0;
    private double t = 0;
    public ArrayList<MaelPose> wayPoints;

    public PathFollower(MaelRobot robot){
        this.robot = robot;
        tracker = robot.tankTracker;
        current = tracker.toPose();
        feed = MaelUtils.feed;
    }
    //ignore, useless
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
    //ignore
    public double[] getWheelVelocities(){
        double omega = Math.toRadians(robot.imu.getYawVelocity()) * getRadius();
        double targetVelocity = omega/getCurvature();

        double left = targetVelocity * (2 + (getCurvature()*distanceBetweenWheels))/2;
        double right = targetVelocity * (2 - (getCurvature()*distanceBetweenWheels))/2;

        double []wheelVelocities = {left,right};

        return wheelVelocities;
    }

    private double getLookAheadSide(){
        MaelPose lookAheadPoint = getLookAheadPoint();
        double xSide = Math.sin(tracker.getHeading())*(lookAheadPoint.x - current.x);
        double ySide = Math.cos(tracker.getHeading()) * (lookAheadPoint.y - current.y);
        double side = Math.signum(xSide - ySide);
        return side;
    }

    public double getCurvature(){
        //double distanceError = (Math.exp(goal.x - initial.x) + Math.exp(goal.y));
        double curvature = (2*(getRobotLine()))/Math.exp(lookAhead);
        return getLookAheadSide()*curvature;
    }

    private double getRobotLine(){
        double a = -Math.tan(tracker.getHeading());
        double b = 1;
        double c = -a * tracker.getX() - tracker.getY();
        MaelPose lookAhead = getLookAheadPoint();
        double x = Math.abs(a*lookAhead.x + b*lookAhead.y + c)/(Math.sqrt((a*a) + (b*b)));
        return x;
    }

    public double getVelocity(){
        return robot.imu.getYawVelocity() * getCurvature();
    }

    public double[] deriveSpeeds(double targetVelocity){
        double r = getRadius();
        double d = distanceBetweenWheels;
        double constantVelocity = (r - (d / 2))/(r + (d / 2));
        if(getCurvature() < 0){
            leftVelocity = targetVelocity;
            rightVelocity = leftVelocity * constantVelocity;
        }
        else if(getCurvature() > 0){
        rightVelocity = targetVelocity;
        leftVelocity = rightVelocity * constantVelocity;
        }
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

        goToPosition(followMe.x,followMe.y,followMe.moveSpeed);

    }

    public void followPath(double velocity){
        double left = deriveSpeeds(velocity)[0];
        double right = deriveSpeeds(velocity)[1];
        double maxRpm = robot.dt.leftDrive.motor1.getRPM();
        robot.dt.leftDrive.setVelocity(left*maxRpm);
        robot.dt.rightDrive.setVelocity(right*maxRpm);
    }


    //ignore, gf stuff
    public void goToPosition(double x, double y, double speed){

        double robotX = tracker.getX();
        double robotY = tracker.getY();
        double robotAngle = tracker.getHeading();
        tracker.update();

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
    //ignore, gf stuff
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

    public MaelPose getLookAheadPoint(){
        MaelPose start = wayPoints.get(0);
        MaelPose center = tracker.toPose();
        MaelPose end = wayPoints.get(wayPoints.size() - 1);

        MaelVector d = new MaelVector(end.x - start.x,end.y - start.y);
        MaelVector f = new MaelVector(start.x - center.x,start.y - center.y);

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
                //return t1;
                t = t1;
            }
            if(t2>= 0 && t2 <= 1){
                t = t2;
            }
        }

        MaelPose lookAheadPoint = new MaelPose(start.x + t*d.x,start.y + t*d.y);
        goal = lookAheadPoint;
        return lookAheadPoint;
    }
    //ignore
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
    //ignore
    public static ArrayList<MaelPose> doubleArrayCopy(ArrayList<MaelPose> arr){

        ArrayList<MaelPose> temp = new ArrayList<>();

        for(int i = 0; i<arr.size(); i++){
            temp.add(arr.get(i));
        }

        return temp;
    }
    //ignore
    public static ArrayList<MaelPose> smoothPath(ArrayList<MaelPose> path, double weightData, double weightSmooth, double tolerance){
        ArrayList<MaelPose> newPath = doubleArrayCopy(path);

        double change = tolerance;
        while(change >= tolerance){
            change = 0.0;
            for(int i=1; i<path.size() - 1; i++){
/*                for(int j = 0; j<path.get(i).x; j++){

                }*/


            }
        }
        return newPath;
    }
    //ignore
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
