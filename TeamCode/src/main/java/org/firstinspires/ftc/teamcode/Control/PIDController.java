package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.robotcore.util.Range;

public class PIDController implements Constants{

    private double i = 0;
    private double d;
    private double error;
    private double power;
    private double KP;
    private double KI;
    private double KD;
    private double previousError = 0;
    private double maxI;
    private double previousTime = 0;
    private double current;
    private double target;



    public PIDController(double KP, double KI, double KD, double maxI) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.maxI = maxI;
    }

    public double power(double lock, double currentLoc) {
        current = currentLoc;
        target = lock;
        error = lock - currentLoc;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECONDS_PER_MINUTE;
        if (Math.abs(currentLoc) > Math.abs(lock) * 0.8){
            i+=error*deltaTime;
        }
        d = (error - previousError)/deltaTime;
        power = (KP*error) + (KI*i) + (KD*d);
        previousTime = System.nanoTime();
        previousError = error;
        return power;
    }

    public double getI () {
        return i;
    }

    public void reset() {
        i = 0;
        previousError = 0;
    }

    public double[] returnVal(){
        double PID[] = {(KP*error),(KI*i),(KD*d),power};
        return PID;
    }

    public double getKp() {
        return KP;
    }

    public double getKi() {
        return KI;
    }

    public double getKd() {
        return KD;
    }

    public void setKp(double kp) {
        this.KP = kp;
    }

    public void setKi(double ki) {
        this.KI = ki;
    }

    public void setKd(double kd) {
        this.KD = kd;
    }

    public double getError() {
        return error;
    }

    public double getCurrent() {
        return current;
    }

    public double getTarget() {
        return target;
    }


}
