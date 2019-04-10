package org.firstinspires.ftc.teamcode.Subsystems.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Leviathan;

import ftc.library.MaelstromControl.MaelstromPID.PIDController;
import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotorSystem;
import ftc.library.MaelstromMotions.MaelstromMotors.MotorModel;
import ftc.library.MaelstromSensors.MaelstromLimitSwitch;
import ftc.library.MaelstromSensors.MaelstromTimer;
import ftc.library.MaelstromUtils.MaelstromUtils;
import ftc.library.MaelstromUtils.TimeConstants;
import ftc.library.MaelstromWrappers.MaelstromController;
import ftc.library.MaelstromWrappers.MaelstromTelemetry;

public class MaelstromPivot implements TimeConstants, Constants {

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
    public MaelstromMotorSystem pivot;
    public MaelstromLimitSwitch limit;
    public MaelstromTelemetry feed;
    public MaelstromUtils.AutonomousOpMode auto;

    private double liftPosition = 0;

    public MaelstromPivot(/*Leviathan robot, */HardwareMap hwMap){
        pivot = new MaelstromMotorSystem("pivot1","pivot2", MotorModel.NEVEREST_NAKED, hwMap);
        pivot.stopAndReset();
        pivot.runWithoutEncoders();
        pivot.setBreakMode();
    }

    public void DriverControl(MaelstromController controller){
        kp = (pivotKP * -liftPosition) + 0.001;
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

        MaelstromTimer timer = new MaelstromTimer();
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
        MaelstromTimer timer = new MaelstromTimer();
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

    public void setLimit(MaelstromLimitSwitch limit){
        this.limit = limit;
    }

    public void setAuto(MaelstromUtils.AutonomousOpMode auto){
        this.auto = auto;
    }

    public void setFeed(MaelstromTelemetry feed){
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
        return PIVOT_LIMIT_ANGLE + (pivot.getAngle()*PIVOT_GEAR_RATIO);
    }

    public void setLiftPosition(double position){
        this.liftPosition = position;
    }

    public boolean opModeActive(){
        return auto.getOpModeIsActive();
    }



}

