package ftc.library.MaelControl.PurePursuit;

/*class for defining a point with x and y coordinates
* along with angle*/
public class MaelPose {
    public double x = 0, y = 0, angle = 0;

    public MaelPose(double x, double y, double angle){
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public MaelPose(double x, double y){
        this(x,y,0);
    }

    public MaelPose(MaelPose thisPose){
        this(thisPose.x,thisPose.y,thisPose.angle);
    }
    public MaelPose(){}

    public MaelVector toVector(){return new MaelVector(x,y);}

    public void setPose(MaelPose pose){
        this.x = pose.x;
        this.y = pose.y;
        this.angle = pose.angle;
    }

    public MaelPose copy(){return new MaelPose(x,y,angle);}
}
