package org.firstinspires.ftc.teamcode.Subsystems.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Control.Constants;

import ftc.library.MaelControl.PID.PIDController;
import ftc.library.MaelMotions.MaelMotors.MaelMotorSystem;
import ftc.library.MaelMotions.MaelMotors.Motor;
import ftc.library.MaelSensors.MaelLimitSwitch;
import ftc.library.MaelSensors.MaelTimer;
import ftc.library.MaelSubsystems.Subsystem;
import ftc.library.MaelUtils.MaelUtils;
import ftc.library.MaelUtils.LibConstants;
import ftc.library.MaelWrappers.MaelController;
import ftc.library.MaelWrappers.MaelTellemetry;

public class MaelPivot implements LibConstants, Constants, Subsystem {

    private double kp = .01, ki = 0, kd = 0;
    private double targetPosition;
    private double basePower = 1;
    private double baseDownPower = 1;
    private double rotatorPower = basePower;
    private double downPower = -1;

    private boolean isReset = false;
    private boolean scoringPosition = false;
    double SCORE_KP = 0.5;

    public PIDController pivotPid = new PIDController(kp,ki,kd,0);
    public MaelMotorSystem pivot;
    public MaelLimitSwitch limit;
    public MaelTellemetry feed;
    public MaelUtils.AutonomousOpMode auto;

    private double liftPosition = 0;
    public State state = State.STOP;

    public enum State{
        UP,
        DOWN,
        STOP
    }

    public State getState(){return state;}

    public void setState(State state){
        this.state = state;
    }

    @Override
    public void update() throws InterruptedException {
        switch (state){
            case UP:
                pivotUp();
                break;
            case DOWN:
                pivotDown();
                break;
            case STOP:
                stop();
                break;
        }
    }

    public MaelPivot(/*Leviathan robot, */HardwareMap hwMap){
        pivot = new MaelMotorSystem("pivot1","pivot2", Motor.NEVEREST_NAKED, hwMap);
        pivot.stopAndReset();
        pivot.runWithoutEncoders();
        pivot.setBreakMode();
    }

    public void DriverControl(MaelController controller){
        kp = (Companion.getPivotKP() * -liftPosition) + 0.001;
        ki = 0.0;
        kd = 0.0;
        if(controller.leftBumper()) pivotUp();
        else if(controller.rightBumper()) pivotDown();
        else stop();

        if(controller.leftBumper() || controller.rightBumper()) targetPosition =getCounts();
        else {
            double currPosition = getCounts();
            double pidPower = pivotPid.power(targetPosition,currPosition);
            pivot.setPower(pidPower);
        }

    }

    public void scoringPosition(){

        PIDController scoringPosition = new PIDController(1,0,0,0);

        MaelTimer timer = new MaelTimer();
        timer.startTime();
        long stopState = 0;

        double desiredAngle = 120;

        while(opModeActive() &&(stopState <= 250)){
            double position = getCounts();
            double power = scoringPosition.power(desiredAngle,getAngle());
            double pidPower = power + (SCORE_KP*Math.sin(getAngle()));

            feed.add("StopState:", timer.stopState());
            feed.add("Angle:", getAngle());
            feed.add("P:", scoringPosition.getP());
            feed.add("I:", scoringPosition.getI());
            feed.add("Error:", scoringPosition.getError());
            feed.add("Angle:", getAngle());
            feed.update();

            pivot.setPower(pidPower);

            if(limit.pressed() && getAngle() <= 0.5) stopState = timer.stopState();
        }
        stop();
    }

    public void downPosition(){
        PIDController downPosition = new PIDController(1,0,0,0);
        MaelTimer timer = new MaelTimer();
        timer.startTime();
        long stopState = 0;

        double desiredAngle = 0;

        while(opModeActive() &&(stopState <= 250)){
            double position = getCounts();
            double power = downPosition.power(desiredAngle,getAngle());
            double pidPower = power + (SCORE_KP*Math.sin(getAngle()));

            feed.add("StopState:", timer.stopState());
            feed.add("Angle:", getAngle());
            feed.add("P:", downPosition.getP());
            feed.add("I:", downPosition.getI());
            feed.add("Error:", downPosition.getError());
            feed.add("Angle:", getAngle());
            feed.update();

            pivot.setPower(pidPower);

            if(limit.pressed() && getAngle() <= 0.5) stopState = timer.stopState();
        }
    }

    public void pivotUp(){
        setPower(rotatorPower);
    }

    public void pivotDown(){
        setPower(downPower);
    }

    public void stop(){
        setPower(0);
    }

    public void setPivotPowers(double up, double down){
        this.rotatorPower = up;
        this.downPower = down;
    }

    public void setLimit(MaelLimitSwitch limit){
        this.limit = limit;
    }

    public void setAuto(MaelUtils.AutonomousOpMode auto){
        this.auto = auto;
    }

    public void setFeed(MaelTellemetry feed){
        this.feed = feed;
    }

    public void setPower(double power){
        pivot.setPower(power);
    }

    public double getPower(){
        return pivot.getPower();
    }

    public void reset(){
        pivot.stopAndReset();
        pivot.runWithoutEncoders();
    }

    public double getCounts(){
        return pivot.getCounts();
    }

    public double getAngle(){
        return Companion.getPIVOT_LIMIT_ANGLE() + (pivot.getAngle()* Companion.getPIVOT_GEAR_RATIO());
    }

    public void setLiftPosition(double position){
        this.liftPosition = position;
    }

    public boolean opModeActive(){
        return auto.getOpModeIsActive();
    }


}

