package ftc.library.MaelControl.PurePursuit;

/*class for defining a point with x and y coordinates
* along with angle*/
public class MaelPose {
    public double x, y, angle;

    public MaelPose(double x, double y, double angle){
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public MaelPose(double x, double y){
        this(x,y,0);
    }


}
