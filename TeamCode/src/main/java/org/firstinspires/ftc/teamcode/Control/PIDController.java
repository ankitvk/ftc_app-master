package org.firstinspires.ftc.teamcode.Control;

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



    public PIDController(double KP, double KI, double KD, double maxI) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.maxI = maxI;
    }

    public double power(double target, double currentLoc) {
        error = target - currentLoc;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECONDS_PER_MINUTE;
        if (Math.abs(currentLoc) > Math.abs(target) * 0.8){
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

}
