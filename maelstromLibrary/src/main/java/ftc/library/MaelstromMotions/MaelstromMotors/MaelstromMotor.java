package ftc.library.MaelstromMotions.MaelstromMotors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelstromControl.MaelstromPID.PIDController;
import ftc.library.MaelstromControl.MaelstromPID.PIDPackage;
import ftc.library.MaelstromSensors.MaelstromEncoder;
import ftc.library.MaelstromSensors.MaelstromLimitSwitch;
import ftc.library.MaelstromSensors.MaelstromTimer;
import ftc.library.MaelstromUtils.MaelstromUtils;
import ftc.library.MaelstromUtils.TimeConstants;

/*custom motor class that includes setting rpm and velocity*/
public class MaelstromMotor implements TimeConstants {
  private DcMotor motor;
  private MaelstromEncoder encoder;
  private PIDPackage pidPackage;
  private MaelstromTimer timer = new MaelstromTimer();
  private int previousPos = 0;
  private double previousRate;
  private long previousTime = 0;
  private double rpm = 0;
  private double power;
  private double motorPower;
  private double KP,KI,KD;
  private double minPower;
  private double targetPower=0;
  private double minPosition=0, maxPosition=0;
  private double previousVelocity=0;
  private double acceleration=0;
  private double previousAcceleration = 0;
  //private double motorCounts = NEVEREST_40_COUNTS_PER_REV;
  private boolean isStalled = false, stallDetection = false,stalled = false;
  private boolean closedLoop, limitDetection = false;
  private Runnable stallAction = new Runnable() {
      @Override
      public void run() {

      }
  },
    unStallAction = new Runnable() {
        @Override
        public void run() {

        }
    };

  private PIDController PID;
  private MaelstromLimitSwitch minLim, maxLim = null;


    public MaelstromMotor(String name, MotorModel type,DcMotor.Direction direction, HardwareMap hwMap){
        motor = hwMap.get(DcMotor.class, name);
        setDirection(direction);
        encoder = new MaelstromEncoder(this,type);
    }
    public MaelstromMotor(String name, MotorModel type,double Kp, double Ki, double Kd, DcMotor.Direction direction, HardwareMap hwMap){
        motor = hwMap.get(DcMotor.class, name);
        setDirection(direction);
        encoder = new MaelstromEncoder(this,type);
        this.PID = new PIDController(Kp,Ki,Kd,1);
    }
    public MaelstromMotor(String name, MotorModel model, HardwareMap hwMap){
        motor = hwMap.get(DcMotor.class, name);
        encoder = new MaelstromEncoder(this,model);
    }

    public void setLimits(MaelstromLimitSwitch min, MaelstromLimitSwitch max){
        minLim = min; maxLim = max;
        limitDetection = true;
    }
    public void setLimits(MaelstromLimitSwitch min){
        minLim = min; maxLim = null;
        limitDetection = true;
    }


    public void init(HardwareMap hwMap, String name){
        motor = hwMap.get(DcMotorEx.class, name);
    }

    public void setPower(double power){
        motor.setPower(power);
    }

    public void setMode(DcMotor.RunMode runMode){
        motor.setMode(runMode);
    }

    public double getRPM(){
        int deltaPos = motor.getCurrentPosition() - previousPos;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_MIN;
        if (deltaTime*6e4 > 10) {
            rpm = (deltaPos/ getCPR())/(deltaTime);
            previousPos = motor.getCurrentPosition();
            previousTime = System.nanoTime();
        }
        return rpm;
    }

    public void setRPM(double rpm){
        power = PID.power(rpm,getRPM());
        motor.setPower((power > 0 && getRPM() < 0) || (power < 0 && getRPM() > 0) ? 0: power);
    }

    public double getVelocity(){
        int deltaPos = getCounts() - previousPos;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_MIN;
        previousPos = getCounts();
        double rate = deltaPos / deltaTime;
        rate = (rate * SECONDS_PER_MIN) / getCPR();
        if(rate != 0) return rate;
        else{
            previousRate = rate;
            return previousRate;
        }
    }

    public void setVelocity(double velocity){
        targetPower = velocity;
        double targetVelocity = encoder.getCPR() * velocity;
        power = PID.power(targetVelocity, getVelocity());
/* motor.setPower((power > 0 && getVelocity() < 0) || (power < 0 && getVelocity() > 0) ? 0: power);*/
        if(!closedLoop) motorPower = targetPower;
        else motorPower = power;
        if(limitDetection){
            if (minLim != null && minLim.pressed() && power < 0 ||
                    maxLim != null && maxLim.pressed() && power > 0)
                motorPower = 0;
            else if (minLim != null && minLim.pressed()
                    && power < 0 && maxLim == null)
                motorPower = 0;
        }
        if(Math.abs(motorPower) < minPower && minPosition != 0) motorPower = 0;
        setPower(motorPower);
    }

    public void setClosedLoop(boolean state){this.closedLoop = state;}

    public double getAcceleration(){
        double deltaPos = getVelocity() - previousVelocity;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_MIN;
        if (deltaTime*6e4 > 10) {
            acceleration = (deltaPos/deltaTime);
            previousVelocity = getVelocity();
            previousTime = System.nanoTime();
        }
        return acceleration;
    }


    public void setPower(double power, MaelstromLimitSwitch lim){
        motorPower = power;
            if (lim != null && lim.pressed() && power < 0) motorPower = 0;
        this.power = power;
        motor.setPower(motorPower);
    }

    public void setSpeed(double speed){
        double rpm = getCPR() * speed;
        power = PID.power(rpm, getRPM());
        motor.setPower((power > 0 && getRPM() < 0) || (power < 0 && getRPM() > 0) ? 0: power);
    }

    public boolean isStalled( double power, int time){
        boolean isStalled = false;
        double prePos = getCounts();
        if ((getCounts() == prePos && getPower() < power )
                && !timer.elapsedTime(time, MaelstromTimer.Time.SECS)) isStalled = true;
        return isStalled;
    }

    public void setKP(double KP){
        this.KP = KP;
    }
    public void setKI(double KI){
        this.KI = KI;
    }
    public void setKD(double KD){
        this.KD = KD;
    }

    public void setPID(double kp, double ki, double kd){
        setKP(kp);
        setKI(ki);
        setKD(kd);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        motor.setZeroPowerBehavior(behavior);
    }

    public void runToPos(int counts, double speed){
        MaelstromTimer timer = new MaelstromTimer();
        stopAndReset();
        motor.setTargetPosition(counts);
        setRunToPos();
        setPower(speed);
        while(isBusy() && timer.elapsedTime(5, MaelstromTimer.Time.SECS)){}
        setPower(0);
        runUsingEncoders();
    }

    public boolean isBusy(){return motor.isBusy();}
    public boolean isRunning(){return getVelocity() > 0;}
    public boolean isPowered(){return getPower() > 0;}

    public boolean isStalled(){
        if(isPowered() && (!isRunning()) && timer.elapsedTime(100, MaelstromTimer.Time.MILLISECS)){
            return isStalled;
        }
        else return !isStalled;
    }

    public boolean getStallDetection(){return stallDetection;}

    public void setStallAction(Runnable action){
        stallAction = action;
    }
    public void setUnStallAction(Runnable action){
        unStallAction = action;
    }
    public void setStallDetection(boolean stall){stallDetection=stall;}

    public void enableStallDetection(){
        setStallDetection(true);
        Runnable main = new Runnable() {
            @Override
            public void run() {
                while (true){
                    stalled = isStalled();
                    if(getStallDetection()){
                        if(stalled) stallAction.run();
                        else unStallAction.run();
                    }
                    MaelstromUtils.sleep(100);
                }
            }
        };
        Thread thread = new Thread(main);
        thread.start();

    }

    public void setBrakeMode(){
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setFloatMode(){
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void runUsingEncoders(){setMode(DcMotor.RunMode.RUN_USING_ENCODER);}
    public void setRunToPos(){setMode(DcMotor.RunMode.RUN_TO_POSITION);}
    public double getAngle (){
        double angle = (360*(motor.getCurrentPosition()) % getEncoder().getCountsPerInch())/ getEncoder().getCPR();
        return angle;
    }

    public void setAngle(double angle){
        double power = PID.power(angle, getAngle());
        motor.setPower(/*(power > 0 && getRPM() < 0) || (power < 0 && getRPM() > 0) ? 0:*/ power);
    }


    public void runWithoutEncoders(){
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopAndReset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getCounts(){return motor.getCurrentPosition();}

    public double getPower(){return motor.getPower();}

    public double getCPR(){return encoder.getCPR();}

    public double getCountsPerInches(){return encoder.getCountsPerInch();}

    public double getInches(){return encoder.getInches();}

    public void setDirection(DcMotor.Direction direction){motor.setDirection(direction);}

    public MaelstromEncoder getEncoder(){
        return encoder;
    }



}
