package ftc.library.MaelMotions.MaelMotors;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

import ftc.library.MaelSensors.MaelLimitSwitch;

/*custom class for creating motor systems*/
public class MaelMotorSystem {
    public MaelMotor motor1,motor2,motor3,motor4;
    private int numMotors;
    private List<MaelMotor> motors;
    private double currPower = 0;
    private double slowPower = 0;
    private String systemName;
    private Motor model;
    public MaelMotorSystem(String name1, String name2, String name3, String name4, double Kp, double Ki, double Kd, HardwareMap hwMap, Motor encoder){
        motor1 = new MaelMotor(name1,encoder, DcMotorSimple.Direction.FORWARD,hwMap);
        motor2 = new MaelMotor(name2,encoder, DcMotorSimple.Direction.FORWARD,hwMap);
        motor3 = new MaelMotor(name3,encoder, DcMotorSimple.Direction.FORWARD,hwMap);
        motor4 = new MaelMotor(name4,encoder, DcMotorSimple.Direction.FORWARD,hwMap);
        motors = Arrays.asList(motor1,motor2,motor3,motor4);
        for(MaelMotor motor : motors) {
            motor.setPID(Kp,Ki,Kd);
        }
        numMotors = 4;
        model = encoder;
    }
    public MaelMotorSystem(String name1, String name2, String name3, String name4, double Kp, double Ki, double Kd, DcMotorSimple.Direction direction, HardwareMap hwMap, Motor encoder){

        motor1 = new MaelMotor(name1,encoder, direction,hwMap);
        motor2 = new MaelMotor(name2,encoder, direction,hwMap);
        motor3 = new MaelMotor(name3,encoder, direction,hwMap);
        motor4 = new MaelMotor(name4,encoder, direction,hwMap);
        motors = Arrays.asList(motor1,motor2,motor3,motor4);
        for(MaelMotor motor : motors) {
            motor.setPID(Kp,Ki,Kd);
    }
        numMotors = 4;
        model = encoder;
    }
    public MaelMotorSystem(String name1, String name2, DcMotorSimple.Direction direction, HardwareMap hwMap, Motor encoder){
        motor1 = new MaelMotor(name1,encoder, direction,hwMap);
        motor2 = new MaelMotor(name2,encoder, direction,hwMap);
        motors = Arrays.asList(motor1,motor2);
        numMotors = 2;
        model = encoder;
    }
    public MaelMotorSystem(String name1, String name2, Motor encoder, HardwareMap hwMap){
        motor1 = new MaelMotor(name1,encoder, hwMap);
        motor2 = new MaelMotor(name2,encoder, hwMap);
        motors = Arrays.asList(motor1,motor2);
        numMotors = 2;
        model = encoder;
    }

    public MaelMotorSystem(String name1, String name2, String name3, double Kp, double Ki, double Kd, DcMotorSimple.Direction direction, HardwareMap hwMap, Motor encoder){
        motor1 = new MaelMotor(name1,encoder, direction,hwMap);
        motor2 = new MaelMotor(name2,encoder, direction,hwMap);
        motor3 = new MaelMotor(name3,encoder, direction,hwMap);
        motors = Arrays.asList(motor1,motor2,motor3);
        for(MaelMotor motor : motors) {
            motor.setPID(Kp,Ki,Kd);
        }
        numMotors = 3;
        model = encoder;
    }

    public MaelMotorSystem(String name1, String name2, double Kp, double Ki, double Kd, DcMotorSimple.Direction direction, HardwareMap hwMap, Motor encoder){
        motor1 = new MaelMotor(name1,encoder, direction,hwMap);
        motor2 = new MaelMotor(name2,encoder, direction,hwMap);
        motors = Arrays.asList(motor1,motor2);
        for(MaelMotor motor : motors) {
            motor.setPID(Kp,Ki,Kd);
        }
        numMotors = 2;
        model = encoder;
    }
    public MaelMotorSystem(String name1, String name2, double Kp, double Ki, double Kd, DcMotorSimple.Direction direction1, DcMotorSimple.Direction direction2, HardwareMap hwMap, Motor encoder){
        motor1 = new MaelMotor(name1,encoder, direction1,hwMap);
        motor2 = new MaelMotor(name2,encoder, direction2,hwMap);
        motors = Arrays.asList(motor1,motor2);
        for(MaelMotor motor : motors) {
            motor.setPID(Kp,Ki,Kd);
        }
        numMotors = 2;
        model = encoder;
    }

    public MaelMotorSystem(String name1, String name2, DcMotorSimple.Direction direction1, DcMotorSimple.Direction direction2, HardwareMap hwMap, Motor encoder){
        motor1 = new MaelMotor(name1,encoder, direction1,hwMap);
        motor2 = new MaelMotor(name2,encoder, direction2,hwMap);
        motors = Arrays.asList(motor1,motor2);
        numMotors = 2;
        model = encoder;
    }

    public MaelMotorSystem setKp(double kp){
        for (MaelMotor motor: motors) motor.setKP(kp);
        return this;
    }
    public MaelMotorSystem setKi(double ki){
        for (MaelMotor motor: motors) motor.setKP(ki);
        return this;
    }
    public MaelMotorSystem setKd(double kd){
        for (MaelMotor motor: motors) motor.setKP(kd);
        return this;
    }
    public MaelMotorSystem setPID(double kp, double ki, double kd){
        for (MaelMotor motor: motors) motor.setPID(kp,ki,kd);
        return this;
    }

    public MaelMotorSystem setGearRatio(double gearRatio){
        for(MaelMotor motor : motors) motor.getEncoder().setGearRatio(gearRatio);
        return this;
    }

    public double getInches(){
        double total = 0;
        for (MaelMotor motor : motors) total += motor.getInches();
        return total / numMotors;
    }

    public void setLimits(MaelLimitSwitch min, MaelLimitSwitch max){
        for(MaelMotor motor : motors){
            motor.setLimits(min,max);
        }
    }

    public void runToPos(int counts, double speed){
        for(MaelMotor motor : motors) motor.runToPos(counts, speed);
    }

    public MaelMotorSystem runWithoutEncoders(){
        for (MaelMotor motor : motors){
            motor.runWithoutEncoders();
        }
        return this;
    }

    public MaelMotorSystem stopAndReset(){
        for(MaelMotor motor : motors){
            motor.stopAndReset();
        }
        return this;
    }

    public double getAngle(){
        double total = 0;
        for (MaelMotor motor : motors) total += motor.getAngle();
        return total / numMotors;
    }

    public void setAngle(double angle){
        for (MaelMotor motor : motors){
            motor.setAngle(angle);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        for(MaelMotor motor : motors){
            motor.setZeroPowerBehavior(behavior);
        }
    }

    public void setMode(DcMotor.RunMode mode){
        for(MaelMotor motor : motors) motor.setMode(mode);
    }

    public void setRunToPos(){
        for(MaelMotor motor : motors){
            motor.setRunToPos();
        }
    }

    public void runWithEncoders(){
        for(MaelMotor motor : motors) motor.runUsingEncoders();
    }

    public void setBreakMode(){
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setFloatMode(){
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public boolean isStalled(){
        boolean isStalled = false;
        for (MaelMotor motor : motors) isStalled = motor.isStalled();
        return isStalled;
    }

    public double getCPR(){
        double total = 0;
        for(MaelMotor motor : motors) total += motor.getCPR();
        return total/numMotors;
    }

    public double getCounts(){
        double total = 0;
        for (MaelMotor motor : motors) total += motor.getCounts();
        return total / numMotors;
    }

    public double getGearRatio(){
        double gearRatio = motor1.getEncoder().getGearRatio();
        return gearRatio;
    }

    public double getWheelCircumference(){
        double circumference = motor1.getEncoder().getWheelCircumference();
        return circumference;
    }

    public double getPower(){
        double total = 0;
        for (MaelMotor motor : motors) total += motor.getPower();
        return total / numMotors;
    }

    public Motor getModel(){
        return model;
    }

    public void setPower(double power){
        for (MaelMotor motor : motors){
            motor.setPower(power);
        }
    }

    public void setPower(double... powers){
        for(double power : powers){
            for(MaelMotor motor : motors){
                motor.setPower(power);
            }
        }
    }

    public double getVelocity(){
        double total = 0;
        for (MaelMotor motor : motors) total += motor.getVelocity();
        return total / numMotors;
    }

    public void setVelocity(double velocity){
        for (MaelMotor motor : motors){
            motor.setVelocity(velocity);
        }
    }

    public void setClosedLoop(boolean state){
        for(MaelMotor motor : motors) motor.setClosedLoop(state);
    }



}
