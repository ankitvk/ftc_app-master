package ftc.library.MaelControl.PurePursuit;

import ftc.library.MaelUtils.LibConstants;
import ftc.library.MaelUtils.MaelMath;

public class MaelVector implements LibConstants {
    public double x,y;

    public MaelVector(double x, double y){
        this.x = x;
        this.y = y;
    }

    public MaelVector(MaelPose point){
        this(point.x,point.y);
    }

    public double getMagnitude(){return Math.hypot(x,y);}

    public double dotProdcut(MaelVector vector){
        return (this.x * vector.x) + (this.y * vector.y);
    }

    public MaelVector multiply(double scalar){
        return new MaelVector(this.x * scalar,this.y * scalar);
    }

    public MaelVector displacement(MaelVector vector){
        return new MaelVector(vector.x - this.x,vector.y - this.y);
    }

    public double distanceToVecotr(MaelVector vector){
        return MaelMath.calculateDistance(this.x,this.y,vector.x,vector.y);
    }

    public double getDirection(){return Math.toDegrees(Math.atan(y/x));}

    public MaelPose toPose(){ return new MaelPose(x,y,getDirection());}
}
