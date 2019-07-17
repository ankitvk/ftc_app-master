package ftc.library.MaelControl.PurePursuit;

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
    public MaelPose current;
    private MaelPose goal;
    private double lookAhead = 1;
    public double trackWidth = 18;
    private double leftVelocity = 0;
    private double rightVelocity = 0;
    private double t = 0;
    private double endKp = 0.04;
    private Path path = new Path();

    public PathFollower(MaelRobot robot){
        this.robot = robot;
        tracker = robot.tankTracker;
        current = tracker.toPose();
        feed = MaelUtils.feed;
    }

    public PathFollower(){
        current = new MaelPose();
    }


    public double getDistance(){
        tracker.update();
        return MaelMath.calculateDistance(goal,tracker.toPose());
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

    public double[] deriveSpeeds(double targetVelocity){
        tracker.update();
        double r = Math.abs(getRadius());
        double d = trackWidth;
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

    public MaelPose getLookAheadPose(){
        MaelPose lookAheadPoint = new MaelPose();

        // iterate through all pairs of points
        for(int i = 0; i < path.numOfPoints() - 1; i++) {
            // form a segment from each two adjacent points
            MaelPose start = path.getPose(i);
            MaelPose end = path.getPose(i + 1);

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
            if (discriminant < 0 /*|| v1 == v2*/ || v1.equals(v2)) continue;

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

    public void followPath(double velocity){

        while(!MaelUtils.linearOpMode.isStopRequested()){
            path.addPoint(current);
            tracker.update();
            double left = deriveSpeeds(velocity)[0];
            double right = deriveSpeeds(velocity)[1];
            robot.dt.leftDrive.setVelocity(-left);
            robot.dt.rightDrive.setVelocity(right);
            MaelPose endPoint = path.endPose();
            double distanceError = MaelMath.calculateDistance(current,endPoint);

           feed.add("X: ",robot.tankTracker.getX());
            feed.add("Y: ",robot.tankTracker.getY());
            feed.add("Heading: ",robot.tankTracker.getHeading());
            feed.add("LookAhead X: ",/*getLookAheadPoint().x*/getLookAheadPose().x);
            feed.add("LookAhead Y: ",/*getLookAheadPoint().y*/getLookAheadPose().y);
            feed.add("Left Power: ",left);
            feed.add("Right Power: ",right);
            feed.update();

            if(distanceError <= 2){
                double targetVel = velocity * (distanceError / lookAhead) * endKp;
                left = deriveSpeeds(targetVel)[0];
                right = deriveSpeeds(targetVel)[1];
                robot.dt.leftDrive.setVelocity(-left);
                robot.dt.rightDrive.setVelocity(right);
            }

            if(distanceError <= .5) break;

            if(MaelUtils.linearOpMode.isStopRequested()) break;
        }
        robot.stop();
    }

    public double getRadius(){
        tracker.update();
        double r = 1 / getCurvature();
        return r;
    }

    public MaelPose getLookAheadPoint(){
        MaelPose lookAheadPoint = new MaelPose();
        for(int i = 0; i < path.numOfPoints() - 1; i++) {
            //MaelMath.lineCircleIntersection()
            tracker.update();
            MaelPose start = path.getPose(i);
            MaelPose end = path.getPose(i + 1);

            MaelVector d = new MaelVector(end.x - start.x, end.y - start.y);
            MaelVector f = new MaelVector(start.x - current.x, start.y - current.y);

            double a = d.dot(d);
            double b = 2 * f.dot(d);
            double c = f.dot(f) - (lookAhead * lookAhead);
            double discriminant = (b * b) - 4 * (a * c);

            if (discriminant < 0) {
                //no intersection
                continue;
            } else {
                discriminant = Math.sqrt(discriminant);
                double t1 = (-b - discriminant) / (2 * a);
                double t2 = (-b + discriminant) / (2 * a);

                if (t1 >= 0 && t1 <= 1) {
                    //return t1;
                    t = t1;
                }
                if (t2 >= 0 && t2 <= 1) {
                    t = t2;
                }
            }


            lookAheadPoint = new MaelPose(start.x + t*d.x,start.y + t*d.y);
            goal = lookAheadPoint;
        }

        if(path.numOfPoints() > 0){
            MaelPose endPoint = path.endPose();

            double endX = endPoint.x;
            double endY = endPoint.y;

            // if we are closer than lookahead distance to the end, set it as the lookahead
            if (Math.sqrt((endX - current.x) * (endX - current.x) + (endY - current.y) * (endY - current.y)) <= lookAhead) {
                return new MaelPose(endX,endY);
            }
        }

        return lookAheadPoint;
    }

    public MaelPose getGoal(){return goal;}

    public void setGoal(MaelPose goal){this.goal = goal;}

    public double getLookAhead(){return lookAhead;}

    public void setLookAhead(double lookAhead){this.lookAhead = lookAhead;}

    public Path getPath(){return path;}

    public void setPath(Path path){this.path = path;}

    public void setEndKp(double kp){this.endKp = kp;}

    public double getTrackWidth(){return trackWidth; }

    public void setTrackWidth(double trackWidth){this.trackWidth = trackWidth;}
}
