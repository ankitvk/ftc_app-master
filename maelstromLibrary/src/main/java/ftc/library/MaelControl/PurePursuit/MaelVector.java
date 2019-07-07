package ftc.library.MaelControl.PurePursuit;

import ftc.library.MaelUtils.LibConstants;
import ftc.library.MaelUtils.MaelMath;

public class MaelVector implements LibConstants {
    public double x,y,z;
    public double velocity, curvature, distance;

    public MaelVector(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
        velocity = 0;
        curvature = 0;
        distance = 0;
    }

    public MaelVector(double x, double y){
        this.x = x;
        this.y = y;
        velocity = 0;
        curvature = 0;
        distance = 0;
    }

    public MaelVector(MaelVector v){
        this.x = v.x;
        this.y = v.y;
        this.z = v.z;
        this.velocity = v.velocity;
        this.curvature = v.curvature;
        this.distance = v.distance;
    }

    public MaelVector(MaelPose point){
        this(point.x,point.y);
    }

    public MaelVector add(MaelVector a, MaelVector b, MaelVector target){
        if (target == null) {
            target = new MaelVector(a.x + b.x, a.y + b.y, a.z + b.z);
        } else {
            target.set(a.x + b.x, a.y + b.y, a.z + b.z);
        }
        return target;
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

    public double distanceToVector(MaelVector vector){
        return MaelMath.calculateDistance(this.x,this.y,vector.x,vector.y);
    }

    public void set(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double getDirection(){return Math.toDegrees(Math.atan(y/x));}

    public MaelPose toPose(){ return new MaelPose(x,y,getDirection());}
}
