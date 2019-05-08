package ftc.library.MaelstromSubsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelstromControl.MaelstromPID.PIDController;
import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotor;
import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotorSystem;
import ftc.library.MaelstromMotions.MaelstromMotors.Motor;
import ftc.library.MaelstromMotions.MaelstromServos.CRServo.MaelstromCRServo;
import ftc.library.MaelstromMotions.MaelstromServos.CRServo.MaelstromCRServoSystem;
import ftc.library.MaelstromUtils.SubsystemModels;
import ftc.library.MaelstromUtils.LibConstants;
import ftc.library.MaelstromWrappers.MaelstromController;

public class MaelstromElevator implements LibConstants,Subsystem {
    public MaelstromMotor oneMotorLift;
    public MaelstromMotorSystem twoMotorLift;
    public MaelstromCRServo oneCrLift;
    public MaelstromCRServoSystem twoCrLift;
    public Motor model;
    public MaelstromController controller;
    public double numOfMotors = 0;
    public double numOfCr = 0;
    public SubsystemModels subModel;
    public double extendPower = 1,retractPower = 1;
    public double kp = 0.01, ki = 0, kd = 0;
    public double counts, power;
    public double targetPosition, currPosition;
    public double spoolDiameter = 0 ;
    public double spoolCircumference = 0 ;
    public double gearboxRatio = 0;
    public PIDController liftPid = new PIDController(kp,ki,kd);
    public State state = State.STOP;
    public enum State{
        EXTEND,
        RETRACT,
        STOP
    }

    public State getState(){return state;}

    public void setState(State state){
        this.state = state;
    }

    @Override
    public void update() throws InterruptedException {
        switch(state){
            case EXTEND:
                extend();
                break;
            case RETRACT:
                retract();
                break;
            case STOP:
                stop();
                break;
        }
    }


    public MaelstromElevator(String name1, Motor model, SubsystemModels subsystem, HardwareMap hwMap){
        if(subsystem == SubsystemModels.MOTOR) {
            oneMotorLift = new MaelstromMotor(name1, model, hwMap);
            this.model = model;
            this.subModel = subsystem;
            numOfMotors = 1;
            reset();
        }
        else if(subsystem == SubsystemModels.CR){
            oneCrLift = new MaelstromCRServo(name1,hwMap);
            this.model = model;
            this.subModel = subsystem;
            numOfCr = 1;
        }
    }

    public MaelstromElevator(String name1, String name2, Motor model, SubsystemModels subsystem, HardwareMap hwMap){
        if(subsystem == SubsystemModels.MOTOR){
            twoMotorLift = new MaelstromMotorSystem(name1,name2,model,hwMap);
            this.model = model;
            this.subModel = subsystem;
            numOfMotors = 2;
            reset();
        }
        else if(subsystem == SubsystemModels.CR){
            twoCrLift = new MaelstromCRServoSystem(name1,name2,hwMap);
            this.model = model;
            this.subModel = subsystem;
            numOfCr = 2;
        }
    }

    public void DriverControl(MaelstromController controller) {
        this.controller = controller;
        if(controller.leftTriggerPressed()) extend();
        else if(controller.rightTriggerPressed()) retract();
        else stop();

        if(subModel == SubsystemModels.MOTOR) liftPid();
    }

    public void liftPid(){
        if(controller.leftTriggerPressed() || controller.rightTriggerPressed()) targetPosition = getCounts();
        else {
            currPosition = getCounts();
            double pidPower = liftPid.power(currPosition, targetPosition);
            setPower(pidPower);
        }
        liftPid.setPID(kp,ki,kd);
    }

    public void extend(){
        setPower(extendPower);
    }

    public void retract(){
        setPower(retractPower);
    }

    public void stop(){
        setPower(0);
    }

    public void setLiftPowers(double extendPower, double retractPower){
        this.extendPower = extendPower;
        this.retractPower = retractPower;
    }

    public void setPower(double power){
        if(numOfMotors == 1) oneMotorLift.setPower(power);
        else if(numOfMotors == 2) twoMotorLift.setPower(power);

        if(numOfCr == 1) oneCrLift.setPower(power);
        else if(numOfCr == 2) twoCrLift.setPower(power);
    }

    public double getPower(){
        if(numOfMotors == 1) power = oneMotorLift.getPower();
        else if(numOfMotors == 2) power = twoMotorLift.getPower();

        if(numOfCr == 1) power = oneCrLift.getPower();
        else if(numOfCr == 2) power = twoCrLift.getPower();
        return power;
    }

    public double getCounts(){
        if(numOfMotors == 1) counts = oneMotorLift.getCounts();
        else if(numOfMotors == 2) counts = twoMotorLift.getCounts();
        return counts;
    }

    public void reset(){
        if(numOfMotors == 1){
            oneMotorLift.stopAndReset();
            oneMotorLift.runWithoutEncoders();
        }
        else if(numOfMotors == 2){
            twoMotorLift.stopAndReset();
            twoMotorLift.runWithoutEncoders();
        }
    }

    public void runToPos(int counts, double speed){
        reset();
        if(numOfMotors ==1){
            oneMotorLift.runToPos(counts,speed);
        }
        else if(numOfMotors ==2) twoMotorLift.runToPos(counts, speed);
    }


    public double getCPR(){
        double cpr = 0;
        if(numOfMotors == 1) cpr += oneMotorLift.getCPR();
        else if(numOfMotors == 2) cpr += twoMotorLift.getCPR();
        return cpr;
    }

    public void setPID(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void setSpoolDiameter(double diameter){
        this.spoolDiameter = diameter;
    }

    public double getSpoolCircumference(){return spoolCircumference = spoolDiameter*Math.PI;}

    public void setGearboxRatio(double ratio){this.gearboxRatio = ratio;}

    public double getGearboxRatio(){return gearboxRatio;}

    public double getDistance(){
        double distance = (getCounts() / getCPR()) * getSpoolCircumference() /  gearboxRatio;
        return distance;

    }


}
