package ftc.library.MaelstromUtils;


public class Delta implements TimeConstants {

    private long prevTime = 0;
    private double deltaTime = 0;
    private double deltaPos = 0;
    private double currTime = 0;
    private double prevPos = 0;
    private double currPos = 0;
    private double setPrevPos = 0;
    private double setCurrPos = 0;

    public Delta(){this.reset();}

    public Delta(double previousPos, double currPos){
        this();
        this.setPrevPos(previousPos);
        this.setCurrPos(currPos);
    }

    public long getNanoSeconds(){return System.nanoTime();}

    public double updateDeltaTime(){
        currTime = getNanoSeconds();
        deltaTime = (currTime - prevTime) / NANOSECS_PER_MIN;
        prevTime = getNanoSeconds();
        return deltaTime;
    }

    public double updatePos(){
        currPos = setCurrPos;
        deltaPos = currPos - prevPos;
        prevPos = setPrevPos;
        return deltaPos;
    }

    public void reset(){
        currTime = 0;
        prevTime = 0;
    }

    public void setPrevPos(double prevPos){this.setPrevPos = prevPos;}

    public void setCurrPos(double pos){this.setCurrPos = pos;}


}
