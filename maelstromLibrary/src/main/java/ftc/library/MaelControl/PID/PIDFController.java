package ftc.library.MaelControl.PID;

import ftc.library.MaelSensors.MaelTimer;
import ftc.library.MaelUtils.LibConstants;

/*class for PID controller*/
public class PIDFController implements LibConstants {
    private double i=0;
    private double d;
    private double error;
    private double power;
    private double kp, ki, kd, kf = 0;
    private double P, I, D, F = 0;
    private double previousError = 0;
    private double maxI;
    private double previousTime = 0;
    private MaelTimer loopTimer = new MaelTimer();

    public PIDFController(double kp, double ki, double kd, double kf, double maxI){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        this.maxI = maxI;

    }
    public PIDFController(double kp, double ki, double kd, double maxI){
        this(kp,ki,kd,0,maxI);
    }

    public PIDFController(double kp, double ki, double kd){
        this(kp,ki,kd,0);
    }

    public PIDFController(double kp, double ki){
        this(kp,ki,0);
    }

    public PIDFController(double kp){
        this(kp,0);
    }


    public double power(double target, double currentLoc){
        double error = target - currentLoc;
        this.error = error;
        P = kp*error;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_MIN/*loopTimer.milliSecs()*/;
        //loopTimer.reset();
        i += Math.abs(currentLoc) > Math.abs(target) * 0.7 ? error*deltaTime : 0;
        I = ki*i;
        if(maxI != 0) i = Math.min(maxI, Math.max(-maxI, i));
        d = (error - previousError)/deltaTime;
        D = kd*d;
        previousTime = System.nanoTime();
        previousError = error;
        F = kf*target;
        //power = (kp*error) + (ki *i) + (kd *d);
        power = P + I + D + F;
        return power;
    }

    public double[] returnVal(){
        double PID[] = {(kp * error), (ki * i), (kd * d), power};
        return PID;
    }

    public double getP(){ return P; }
    public double getI(){ return I; }
    public double getD(){ return D; }
    public double getF(){return F;}

    public double getError(){return Math.abs(error);}

    public double getIntegral() {
        return i;
    }

    public double getKp() {
        return kp;
    }
    public double getKi() {
        return ki;
    }
    public double getKd() {
        return kd;
    }
    public double getKf(){return kf;}

    public void setKP(double KP){this.kp = KP;}
    public void setKI(double KI){this.ki = KI;}
    public void setKD(double KD){this.kd = KD;}
    public void setKF(double KF){this.kd = KF;}

    public void setPID(double kp, double ki, double kd, double kf){
        setKP(kp);
        setKI(ki);
        setKD(kd);
        setKF(kp);
    }

    public void setPID(double kp, double ki, double kd){
        setPID(kp,ki,kd,0);
    }

    public void setPID(double kp, double ki){
        setPID(kp,ki,0);
    }

    public void setPID(double kp){
        setPID(kp,0);
    }

    public void reset(){
        i = 0;
        previousError = 0;
    }


}

