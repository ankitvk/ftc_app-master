package ftc.library.MaelMotions.MaelMotors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelControl.PID.PIDController;
import ftc.library.MaelSensors.MaelTimer;
import ftc.library.MaelSensors.MaelstromEncoder;
import ftc.library.MaelSensors.MaelLimitSwitch;
import ftc.library.MaelUtils.MaelUtils;
import ftc.library.MaelUtils.LibConstants;
import ftc.library.MaelUtils.TimeUnits;

/*custom motor class that includes setting rpm and velocity*/
public class MaelMotor implements LibConstants {
    private DcMotor motor;
  private MaelstromEncoder encoder;
  private MaelTimer timer = new MaelTimer();
  private int previousPos = 0;
  private double previousRate;
  private double currVelocity = 0;
  private long previousTime = 0;
  private double rpm = 0;
  private double power = 0;
  private double motorPower = 0;
  private double KP,KI,KD;
  private double minPower = 0;
  private double minPosition=0, maxPosition=0;
  private double previousVelocity=0;
  private double acceleration=0;
  private double pidPower = 0;
  private double previousAcceleration = 0;
  //private double motorCounts = NEVEREST_40_COUNTS_PER_REV;
  private boolean isStalled = false, stallDetection = false,stalled = false;
  private boolean closedLoop = false, limitDetection = false;
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

  private PIDController PID = new PIDController(KP,KI,KD);
  private MaelLimitSwitch minLim, maxLim = null;


    public MaelMotor(String name, Motor type, DcMotor.Direction direction, HardwareMap hwMap){
        motor = hwMap.get(DcMotor.class, name);
        setDirection(direction);
        encoder = new MaelstromEncoder(this,type);
    }
    public MaelMotor(String name, Motor type, double Kp, double Ki, double Kd, DcMotor.Direction direction, HardwareMap hwMap){
        motor = hwMap.get(DcMotor.class, name);
        setDirection(direction);
        encoder = new MaelstromEncoder(this,type);
        this.KP = Kp;
        this.KI = Ki;
        this.KD = Kd;
    }
    public MaelMotor(String name, Motor model, HardwareMap hwMap){
        motor = hwMap.get(DcMotor.class, name);
        encoder = new MaelstromEncoder(this,model);
    }

    public void setLimits(MaelLimitSwitch min, MaelLimitSwitch max){
        minLim = min; maxLim = max;
        limitDetection = true;
    }
    public void setLimits(MaelLimitSwitch min){
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
/*        int deltaPos = getCounts() - previousPos;
        //double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_SEC;
        double deltaTime = MaelUtils.getDeltaTime();
        if (deltaTime*6e4 > 10) {
            currVelocity = (deltaPos/ getCPR())/(deltaTime);
            previousPos = motor.getCurrentPosition();
            previousTime = System.nanoTime();
        }
        return currVelocity;*/
        int deltaPos = getCounts() - previousPos;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_SEC;
        //double deltaTime = MaelUtils.getDeltaTime();
        if (deltaTime*6e4 > 10) {
            currVelocity = (deltaPos/ getCPR())/(deltaTime);
            previousPos = motor.getCurrentPosition();
            previousTime = System.nanoTime();
        }
        return currVelocity;
    }

    public double getTargetVelocity(double velocity){
        double target = encoder.getRPM() * velocity;
        return target;
    }

    public void setVelocity(double velocity){
        double targetVelocity = getTargetVelocity(velocity);
        if(closedLoop){ pidPower = PID.power(targetVelocity,getVelocity());}
        else {pidPower = velocity;}
        /*if(limitDetection){
            if (minLim != null && minLim.pressed() && power < 0 ||
                    maxLim != null && maxLim.pressed() && power > 0)
                motorPower = 0;
            else if (minLim != null && minLim.pressed()
                    && power < 0 && maxLim == null)
                motorPower = 0;
        }*/
        setPower(pidPower);
    }

    public double getPidPower(){return pidPower;}

    public void setClosedLoop(boolean state){this.closedLoop = state;}

    public double getAcceleration(){
        double deltaPos = getVelocity() - previousVelocity;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_SEC;
        if (deltaTime*6e4 > 10) {
            acceleration = (deltaPos/deltaTime);
            previousVelocity = getVelocity();
            previousTime = System.nanoTime();
        }
        return acceleration;
    }


    public void setPower(double power, MaelLimitSwitch lim){
        motorPower = power;
            if (lim != null && lim.pressed() && power < 0) motorPower = 0;
        this.power = power;
        motor.setPower(motorPower);
    }

    public void setSpeed(double speed){
        double rpm = getRPM() * speed;
        power = PID.power(rpm, getRPM());
        motor.setPower((power > 0 && getRPM() < 0) || (power < 0 && getRPM() > 0) ? 0: power);
    }

    public boolean isStalled( double power, int time){
        boolean isStalled = false;
        double prePos = getCounts();
        if ((getCounts() == prePos && getPower() < power )
                && !timer.elapsedTime(time, TimeUnits.SECS)) isStalled = true;
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
        MaelTimer timer = new MaelTimer();
        stopAndReset();
        motor.setTargetPosition(counts);
        setRunToPos();
        setPower(speed);
        while(isBusy() && timer.elapsedTime(5, TimeUnits.SECS)){}
        setPower(0);
        runUsingEncoders();
    }

    public boolean isBusy(){return motor.isBusy();}
    public boolean isRunning(){return getVelocity() > 0;}
    public boolean isPowered(){return getPower() > 0;}

    public boolean isStalled(){
        if(isPowered() && (!isRunning()) && timer.elapsedTime(100, TimeUnits.SECS)){
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
                    MaelUtils.sleep(100);
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
