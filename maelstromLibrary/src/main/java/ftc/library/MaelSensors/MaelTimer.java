package ftc.library.MaelSensors;

import com.qualcomm.robotcore.util.ElapsedTime;

import ftc.library.MaelUtils.LibConstants;
import ftc.library.MaelUtils.TimeUnits;

/*Custom class for Timer*/
public class MaelTimer implements LibConstants {

    private long startTime;
    private long stopState;
    private boolean isPaused = false;
    private long pauseStart;
    private long pauseLength;
    private long nanoSecs;
    private String name = "TIMER";
   private ElapsedTime timer = new ElapsedTime();

    public MaelTimer(){this.reset();}

    public void reset(){
        isPaused = false;
        pauseStart = 0L;
        pauseLength = 0L;
        startTime = 0L;
    }

    public long nanoSecs() {
        nanoSecs = System.nanoTime() - startTime - pauseLength;
        return nanoSecs;
    }
    public double milliSecs() {return nanoSecs() * NANOSECS_PER_MILISEC;}
    public double secs(){return milliSecs() / NANOSECS_PER_MILISEC;}


    public boolean elapsedTime(double time, TimeUnits type){
        return nanoSecs() > (long)(time*type.value);
    }

    public long startTime(){
        startTime = System.nanoTime();
        //this.startTime = startTime;
        return startTime;
    }

    public long stopState(){
        stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
        return stopState;
    }

    public void pause(){
        if (!isPaused) pauseStart = System.nanoTime();
        isPaused = true;
    }

    public void resume(){
        if (isPaused) pauseLength += (System.nanoTime() - pauseStart);
        isPaused = false;
    }

    public boolean isPause(){return isPaused;}
    public String getName(){return name;}


}
