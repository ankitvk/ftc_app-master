package ftc.library.MaelControl.PurePursuit;

import java.util.ArrayList;

import ftc.library.MaelControl.PurePursuit.warriorlib.Path;
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
    private MaelPose goal;
    private double lookAhead = 1;
    public double distanceBetweenWheels = 18;
    double leftVelocity = 0;
    double rightVelocity = 0;
    private double r = 0;
    public double distanceError = 0;
    private double t = 0;
    private double endKp = 0.04;
    private Path path = new Path();

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
        tracker.update();

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
        tracker.update();
        MaelPose lookAheadPoint = /*getLookAheadPoint()*/getLookAheadPose();
        double xSide = Math.sin(tracker.getHeading())*(lookAheadPoint.x - current.x);
        double ySide = Math.cos(tracker.getHeading()) * (lookAheadPoint.y - current.y);
        double side = Math.signum(xSide - ySide);
        return side;
    }

    public double getCurvature(){
        tracker.update();
        //double distanceError = (Math.exp(goal.x - initial.x) + Math.exp(goal.y));
        double curvature = (2*(getRobotLine()))/Math.pow(lookAhead,2);
        return getLookAheadSide()*curvature;
    }

    private double getRobotLine(){
        tracker.update();
        double a = -Math.tan(tracker.getHeading());
        double b = 1;
        double c = -a * tracker.getX() - tracker.getY();
        MaelPose lookAhead = /*getLookAheadPoint()*/getLookAheadPose();
        double x = Math.abs(a*lookAhead.x + b*lookAhead.y + c)/(Math.sqrt((a*a) + (b*b)));
        return x;
    }

    public double getVelocity(){
        return robot.imu.getYawVelocity() * getCurvature();
    }

    public double[] deriveSpeeds(double targetVelocity){
        tracker.update();
        double r = Math.abs(getRadius());
        double d = distanceBetweenWheels;
        double constantVelocity = (r - (d / 2))/(r + (d / 2));
        if(getCurvature() < 0){
            leftVelocity = targetVelocity;
            rightVelocity = leftVelocity * constantVelocity;
        }
        if(getCurvature() > 0){
        rightVelocity = targetVelocity;
        leftVelocity = rightVelocity * constantVelocity;
        }
        /*rightVelocity = targetVelocity;
        leftVelocity = rightVelocity * constantVelocity;*/
        double speeds[] = {leftVelocity,rightVelocity};
        //MaelUtils.normalizeValues(speeds);
        return speeds;
    }

    public MaelPose getFollow(){
        MaelPose followMe = path.startPose();

        for(int i = 0; i < path.getPath().size(); i++){
            MaelPose startPoint = path.startPose();
            MaelPose endPoint = path.endPose();

            ArrayList<MaelPose> intersections = MaelMath.lineCircleIntersection(current,lookAhead,startPoint,endPoint);

            double closestAngle = 1000000;

            for(MaelPose thisIntersection: intersections){

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

    //ignore, gf stuff
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
    //ignore, gf stuff
    public void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
        CurvePoint followMe = getFollowPoint(allPoints,tracker.toPose(),
                allPoints.get(0).followDistance);

        goToPosition(followMe.x,followMe.y,followMe.moveSpeed);

    }



    private MaelPose getLookAheadPose(){
        MaelPose lookAheadPoint = new MaelPose();

        // iterate through all pairs of points
        for(int i = 0; i < path.numOfPoints() - 1; i++) {
            // form a segment from each two adjacent points
            MaelPose start = path.getPose(i);
            MaelPose end = path.getPose(i + i);

            // translate the segment to the origin
            MaelVector v1 = new MaelVector(start.x - current.x, start.y - current.y);
            MaelVector v2 = new MaelVector(end.x - current.x, end.y - current.y);

            // calculate an intersection of a segment and a circle with radius r (lookahead) and origin (0, 0)
            double dx = v2.x - v1.x;
            double dy = v2.y - v1.y;
            double d = Math.sqrt(dx * dx + dy * dy);
            double D = v1.x * v2.y - v2.x * v1.y;

            // if the discriminant is zero or the points are equal, there is no intersection
            double discriminant = lookAhead * lookAhead * d * d - D * D;
            if (discriminant < 0 /*|| v1 == v2*/ || v1 == v2) continue;

            // the x components of the intersecting points
            double x1 = (D * dy + Math.signum(dy) * dx * Math.sqrt(discriminant)) / (d * d);
            double x2 = (D * dy - Math.signum(dy) * dx * Math.sqrt(discriminant)) / (d * d);

            // the y components of the intersecting points
            double y1 = (-D * dx + Math.abs(dy) * Math.sqrt(discriminant)) / (d * d);
            double y2 = (-D * dx - Math.abs(dy) * Math.sqrt(discriminant)) / (d * d);


            // whether each of the intersections are within the segment (and not the entire line)
            boolean validIntersection1 = Math.min(v1.x, v2.x) < x1 && x1 < Math.max(v1.x, v2.x)
                    || Math.min(v1.y, v2.y) < y1 && y1 < Math.max(v1.y, v2.y);
            boolean validIntersection2 = Math.min(v1.x, v2.x) < x2 && x2 < Math.max(v1.x, v2.x)
                    || Math.min(v1.y, v2.y) < y2 && y2 < Math.max(v1.y, v2.y);

            // remove the old lookahead if either of the points will be selected as the lookahead
            if (validIntersection1 || validIntersection2) lookAheadPoint = null;

            // select the first one if it's valid
            if (validIntersection1) {
                lookAheadPoint = new MaelPose(x1 + current.x, y1 + current.y);

            }

            // select the second one if it's valid and either lookahead is none,
            // or it's closer to the end of the segment than the first intersection
            if (validIntersection2) {
                if (lookAheadPoint == null || Math.abs(x1 - v2.x) > Math.abs(x2 - v2.x) || Math.abs(y1 - v2.y) > Math.abs(y2 - v2.y)) {
                    lookAheadPoint = new MaelPose(x2 + current.x, y2 + current.y);
                }
            }
        }

        // special case for the very last point on the path
        if(path.numOfPoints() > 0){
            MaelPose endPoint = path.endPose();

            double endX = endPoint.x;
            double endY = endPoint.y;

            // if we are closer than lookahead distance to the end, set it as the lookahead
            if (Math.sqrt((endX - current.x) * (endX - current.x) + (endY - current.y) * (endY - current.y)) <= lookAhead) {
                    return new MaelPose(endX,endY);
            }
        }
        goal = lookAheadPoint;
        return lookAheadPoint;
    }


/*    private float[] getLookaheadPoint(float x, float y, float r) {
        float[] lookahead = null;

        // iterate through all pairs of points
        for (int i = 0; i < path.size() - 1; i++) {
            // form a segment from each two adjacent points
            float[] segmentStart = path.get(i);
            float[] segmentEnd = path.get(i + 1);

            // translate the segment to the origin
            float[] p1 = new float[]{segmentStart[0] - x, segmentStart[1] - y};
            float[] p2 = new float[]{segmentEnd[0] - x, segmentEnd[1] - y};

            // calculate an intersection of a segment and a circle with radius r (lookahead) and origin (0, 0)
            float dx = p2[0] - p1[0];
            float dy = p2[1] - p1[1];
            float d = (float) Math.sqrt(dx * dx + dy * dy);
            float D = p1[0] * p2[1] - p2[0] * p1[1];

            // if the discriminant is zero or the points are equal, there is no intersection
            float discriminant = r * r * d * d - D * D;
            if (discriminant < 0 || Arrays.equals(p1, p2)) continue;

            // the x components of the intersecting points
            float x1 = (float) (D * dy + signum(dy) * dx * Math.sqrt(discriminant)) / (d * d);
            float x2 = (float) (D * dy - signum(dy) * dx * Math.sqrt(discriminant)) / (d * d);

            // the y components of the intersecting points
            float y1 = (float) (-D * dx + Math.abs(dy) * Math.sqrt(discriminant)) / (d * d);
            float y2 = (float) (-D * dx - Math.abs(dy) * Math.sqrt(discriminant)) / (d * d);

            // whether each of the intersections are within the segment (and not the entire line)
            boolean validIntersection1 = Math.min(p1[0], p2[0]) < x1 && x1 < Math.max(p1[0], p2[0])
                    || Math.min(p1[1], p2[1]) < y1 && y1 < Math.max(p1[1], p2[1]);
            boolean validIntersection2 = Math.min(p1[0], p2[0]) < x2 && x2 < Math.max(p1[0], p2[0])
                    || Math.min(p1[1], p2[1]) < y2 && y2 < Math.max(p1[1], p2[1]);

            // remove the old lookahead if either of the points will be selected as the lookahead
            if (validIntersection1 || validIntersection2) lookahead = null;

            // select the first one if it's valid
            if (validIntersection1) {
                lookahead = new float[]{x1 + x, y1 + y};
            }

            // select the second one if it's valid and either lookahead is none,
            // or it's closer to the end of the segment than the first intersection
            if (validIntersection2) {
                if (lookahead == null || Math.abs(x1 - p2[0]) > Math.abs(x2 - p2[0]) || Math.abs(y1 - p2[1]) > Math.abs(y2 - p2[1])) {
                    lookahead = new float[]{x2 + x, y2 + y};
                }
            }
        }

        // special case for the very last point on the path
        if (path.size() > 0) {
            float[] lastPoint = path.get(path.size() - 1);

            float endX = lastPoint[0];
            float endY = lastPoint[1];

            // if we are closer than lookahead distance to the end, set it as the lookahead
            if (Math.sqrt((endX - x) * (endX - x) + (endY - y) * (endY - y)) <= r) {
                return new float[]{endX, endY};
            }
        }

        return lookahead;
    }*/


    public void write(String m, Object v){
        System.out.println(m + v);
    }

    public void followPath(double velocity){
        while(!MaelUtils.linearOpMode.isStopRequested()){
            path.addPoint(current);
            tracker.update();
            double maxRpm = robot.dt.fl.getRPM();
            double left = deriveSpeeds(velocity)[0]/* / maxRpm*/;
            double right = deriveSpeeds(velocity)[1]/* / maxRpm*/;
            robot.dt.leftDrive.setVelocity(-left);
            robot.dt.rightDrive.setVelocity(right);
            MaelPose endPoint = path.endPose();
            distanceError = MaelMath.calculateDistance(current,endPoint);
            double initialVelocity = velocity;

            //System.out.println("");

           feed.add("X: ",robot.tankTracker.getX());
            feed.add("Y: ",robot.tankTracker.getY());
            feed.add("Heading: ",robot.tankTracker.getHeading());
            feed.add("LookAhead X: ",/*getLookAheadPoint().x*/getLookAheadPose().x);
            feed.add("LookAhead Y: ",/*getLookAheadPoint().y*/getLookAheadPose().y);
            feed.add("Left Power: ",left);
            feed.add("Right Power: ",right);
            feed.update();

/*            write("X: ",robot.tankTracker.getX());
            write("Y: ",robot.tankTracker.getY());
            write("Heading: ",robot.tankTracker.getHeading());
            write("LookAhead X: ",getLookAheadPoint().x);
            write("LookAhead Y: ",getLookAheadPoint().y);
            write("Left Power ",left);
            write("Right Power ",right);*/
            //feed.update();

            //if(distanceError <= 5) break;

            try{
                Thread.sleep(1);
            }
            catch (InterruptedException e){
                e.printStackTrace();
            }

            if(MaelUtils.linearOpMode.isStopRequested()) break;
        }
/*        if(distanceError <= lookAhead){
           double targetVelocity  = initialVelocity*(distanceError / lookAhead) * endKp;
            robot.dt.leftDrive.setVelocity(targetVelocity);
            robot.dt.rightDrive.setVelocity(targetVelocity);
        }*/
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
        tracker.update();
        double r = 1 / getCurvature();
        return r;
    }

    public MaelPose getLookAheadPoint(){
        //MaelMath.lineCircleIntersection()
        tracker.update();
        MaelPose start = path.startPose();
        MaelPose center = tracker.toPose();
        MaelPose end = path.endPose();

        MaelVector d = new MaelVector(end.x - start.x,end.y - start.y);
        MaelVector f = new MaelVector(start.x - center.x,start.y - center.y);

        double a = d.dot(d);
        double b = 2 * f.dot(d);
        double c = f.dot(f) - (lookAhead * lookAhead);
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
/*                for(int j = 0; j<path.getPose(i).x; j++){

                }*/
            }
        }
        return newPath;
    }

    //ignore
    public double[][] r(double[][] path, double weight_data, double weight_smooth, double tolerance)
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

    public static double[][] smoother(double[][] path, double weight_data, double weight_smooth, double tolerance)
    {

        //copy array
        double[][] newPath = path.clone();

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
        //System.out.println(Arrays.deepToString(newPath));
        return newPath;
    }



    public MaelPose getGoal(){return goal;}

    public void setGoal(MaelPose goal){this.goal = goal;}

    public double getLookAhead(){return lookAhead;}

    public void setLookAhead(double lookAhead){this.lookAhead = lookAhead;}

    public Path getPath(){return path;}

    public void setPath(Path path){this.path = path;}
}
