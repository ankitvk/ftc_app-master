package ftc.library.MaelControl.PurePursuit.warriorlib;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelControl.PurePursuit.MaelVector;

public class Path {
    private ArrayList<MaelPose> path = new ArrayList<>();

    public Path(/*ArrayList<MaelPose> path*/){
        this.path = path;
    }

    public double getX(int index){return path.get(index).x;}
    public double getY(int index){return path.get(index).y;}
    public double getAngle(int index){return path.get(index).angle;}

    public ArrayList<MaelPose> getPath(){return path;}

    public MaelPose getStartPoint(){return path.get(0);}

    public MaelPose getEndPoint(){return path.get(path.size() - 1);}

    public void addPoint(MaelPose point){
        path.add(point);
    }

    public void addPoints(MaelPose... points){
        path.addAll(Arrays.asList(points));
    }

    public Path reverse(){
        ArrayList<MaelPose> temp = path;
        Collections.reverse(temp);
        Path reversed = new Path();
        for(int i = 0; i < temp.size() - 1; i++){
            reversed.addPoint(temp.get(i));
        }
        return reversed;
    }



}
