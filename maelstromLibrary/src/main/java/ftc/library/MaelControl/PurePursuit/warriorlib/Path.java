package ftc.library.MaelControl.PurePursuit.warriorlib;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelControl.PurePursuit.MaelVector;

public class Path {
    private ArrayList<MaelPose> path = new ArrayList<>();
    private double spacing = .5;

    public Path(ArrayList<MaelPose> path){
        this.path = path;
    }

    public Path(){}

    public double getX(int index){return path.get(index).x;}
    public double getY(int index){return path.get(index).y;}
    public double getAngle(int index){return path.get(index).angle;}

    public ArrayList<MaelPose> getPath(){return path;}

    public double getSpacing(){return spacing;}

    public void setSpacing(double spacing){this.spacing = spacing;}

    public void addPath(ArrayList<MaelPose> array){this.path = array;}

    public MaelPose startPose(){return path.get(0);}

    public MaelPose endPose(){return path.get(path.size() - 1);}

    public void addPoint(MaelPose point){
        path.add(point);
    }

    public void addPoints(MaelPose... points){
        path.addAll(Arrays.asList(points));
    }


    public void reverse(){
        ArrayList<MaelPose> temp = path;
        Collections.reverse(temp);
        addPath(temp);
    }

    public MaelPose getPose(int i){
        return path.get(i);
    }

    public Path copy(){
        Path temp = new Path();
/*        for(int i = 0; i < numOfPoints() ; i++){
            temp.addPoint(getPose(i));
        }*/
        temp.addPath(path);
        return temp;
    }

    public void clear(){
        path.clear();
        addPath(path);
    }

    public double numOfPoints(){return path.size();}

    public void injectPoints(){
        ArrayList<MaelPose> temp = new ArrayList<>();
        for(int i = 0; i < numOfPoints() - 1; i++) {
            MaelVector start = getPose(i).toVector();
            MaelVector end = getPose(i + 1).toVector();
            MaelVector v = new MaelVector(end.x - start.x, end.y - start.y);
            double pointsCount = Math.ceil(v.norm() / spacing);
            MaelVector unit = v.normalize(null);
            unit.mult(v.norm() / pointsCount);
            for (int j = 0; j < pointsCount; j++) {
                MaelVector newVector = MaelVector.mult(unit, j, null);
                temp.add(MaelVector.add(start, newVector, null).toPose());
            }
        }
        temp.add(endPose());
        addPath(temp);
    }
    /*For path smoothing, a = 1-b and both a and b should be between 0 and 1.
    Ideally b should fall in the 0.7-0.9 range.
    The higher the b value, the smoother the path.
spacing is the distance between points along the path: the path generator will inject points along the coordinates you specify, at the given spacing.
tolerance is the maximum change per single smoothing iteration. If tolerance is exceeded, the smoothing algorithm runs again. This repeats until the change in the path is below the tolerance, which should be set around 0.001. If this value is too low, the smoothing might never converge, so try raising the tolerance if this occurs.*/

    /*smooths path.
     * a and b should be between 0 and 1
     * a = 1-b
     * b (higher = more smooth, 0.7 - 0.9)
     * tolerance convergence tolerance amount (higher = less smoothing)
     * */

/*    public void smooth(double a, double b, double tolerance) {
        ArrayList<Vector> newPath = new ArrayList<>();
        for (Vector v : robotPath) {
            newPath.add(new Vector(v));
        }
        double change = tolerance;
        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < robotPath.size() - 1; ++i) {
                Vector oldVec = robotPath.get(i);
                Vector currVec = newPath.get(i);
                Vector currVecCopy = new Vector(currVec);
                Vector prevVec = newPath.get(i - 1);
                Vector nextVec = newPath.get(i + 1);
                currVec.x += a * (oldVec.x - currVec.x) + b * (prevVec.x + nextVec.x - 2 * currVec.x);
                currVec.y += a * (oldVec.y - currVec.y) + b * (prevVec.y + nextVec.y - 2 * currVec.y);
                change += Math.abs(currVecCopy.x - currVec.x);
                change += Math.abs(currVecCopy.y - currVec.y);
            }
        }
        ArrayList<Vector> path = new ArrayList<>();
        for (Vector v : newPath)
            path.add(new Vector(v));
        robotPath = path;
    }*/

    public void smooth(double a, double b, double tolerance){
        ArrayList<MaelPose> newPath = new ArrayList<>();

        for(MaelPose p : path){
            newPath.add(p);
        }
        double change = tolerance;
        while (change >= tolerance){
            change = 0.0;
            for(int i = 1; i < numOfPoints() - 1; i++){
                MaelPose oldVec = path.get(i);
                MaelPose currVec = newPath.get(i);
                MaelPose currVecCopy = new MaelPose(currVec);
                MaelPose prevVec = newPath.get(i - 1);
                MaelPose nextVec = newPath.get(i + 1);
                currVec.x += a * (oldVec.x - currVec.x) + b * (prevVec.x + nextVec.x - 2 * currVec.x);
                currVec.y += a * (oldVec.y - currVec.y) + b * (prevVec.y + nextVec.y - 2 * currVec.y);
                change += Math.abs(currVecCopy.x - currVec.x);
                change += Math.abs(currVecCopy.y - currVec.y);
            }
        }
        ArrayList<MaelPose> path = new ArrayList<>();
        for (MaelPose p : newPath) {
            path.add(new MaelPose(p));
        }
        this.path = path;
    }

    /*public Path smooth(double a, double b, double tolerance){

        Path newPath = copy();

        double change = tolerance;
        while(change >= tolerance){
            change = 0.0;
            for(int i = 1; i< numOfPoints() - 1; i++){
                MaelPose oldPose = getPose(i);
                MaelPose currPose = newPath.getPose(i);
                MaelPose currPoseCopy = currPose.copy();
                MaelPose prevPose = newPath.getPose(i - 1);
                MaelPose nextPose = newPath.getPose(i + 1);
                currPose.x += a * (oldPose.x - currPose.x) + b * (prevPose.x + nextPose.x - 2 * currPose.x);
                currPose.y += a * (oldPose.y - currPose.y) + b * (prevPose.y + nextPose.y - 2 * currPose.y);
                change += Math.abs(currPoseCopy.x - currPose.x);
                change += Math.abs(currPoseCopy.y - currPose.y);
            }
        }
        return newPath;
    }*/
}
