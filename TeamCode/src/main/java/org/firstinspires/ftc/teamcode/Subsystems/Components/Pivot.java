package org.firstinspires.ftc.teamcode.Subsystems.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.Controller;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class Pivot implements Constants {

    private double kp = 0.01;
    private double ki = 0;
    private double kd = 0;

    private double targetPosition;
    private double basePower = 1;
    private double baseDownPower = 1;
    private double rotatorPower = basePower;
    private double downPower = -0.1;


    public PIDController pivotControl = new PIDController(kp,ki,kd,1);

    private double liftPosition = 0;

    Hardware hardware;
    private AutonomousOpMode auto;
    private Telemetry telemetry;

    public Pivot(Hardware hardware){
        this.hardware=hardware;
        auto = hardware.auto;
        telemetry = hardware.telemetry;
    }

    public void driverControl(Gamepad controller) {
        kp = (pivotKP * -liftPosition) + 0.001;
        ki = 0.0;
        kd = 0.0;
        rotatorPower = (0.0001 * -liftPosition) + basePower;
        downPower = (0.001 * Math.abs(getPosition()) + baseDownPower);
        if (controller.left_bumper){
            setPower(-rotatorPower);
        }
        else if (controller.right_bumper) {
            setPower(downPower);
        } else if (controller.b) {
            setAngle(90);
        } else {
            setPower(0);
        }

        if (controller.right_bumper|| controller.left_bumper) targetPosition = getPosition();
        else {
            double currentPosition = getPosition();
            //setPower(pivotControl.power(targetPosition,currentPosition));
        }

        /*if(!hardware.limit.getState()){
            eReset();
        }*/

        pivotControl.setKp(kp);
        pivotControl.setKi(ki);
        pivotControl.setKd(kd);
    }

    public void driverControlSingle(Gamepad controller) {
        kp = (pivotKP * -liftPosition) + 0.001;
        ki = 0.0;
        kd = 0.0;
        rotatorPower = (0.0005 * -liftPosition) + basePower;
        downPower = (0.005 * Math.abs(getPosition()) + baseDownPower);
        if (controller.dpad_right){
            setPower(-rotatorPower);
        }
        else if (controller.dpad_left) {
            setPower(downPower);
        } else {
            setPower(0);
        }

        if (controller.dpad_left|| controller.dpad_right)
            targetPosition = getPosition();
        else {
            double currentPosition = getPosition();
            //setPower(pivotControl.power(targetPosition,currentPosition));
        }

/*        if(!hardware.limit.getState()){
            eReset();
        }*/

        pivotControl.setKp(kp);
        pivotControl.setKi(ki);
        pivotControl.setKd(kd);
    }

    public void setAngle(double angle){
        PIDController pivotAngleController = new PIDController(Math.abs(Math.cos(getAngle()))*0.006 + 0.05 ,0,0,0);

        double power = pivotAngleController.power(angle, getAngle());

        hardware.pivot1.setPower(power);
        hardware.pivot2.setPower(power);

    }

    public void scoringPosition(){
            PIDController scoringPosition = new PIDController(1,0,0,0);
            long startTime = System.nanoTime();
            long beginTime = startTime;
            long stopState = 0;

            double ticks = 0;

            while((opModeIsActive() && (stopState <= 250))){
                double position = getPosition();
                double power = scoringPosition.power(ticks,position);

                telemetry.addData("stopstate: ", stopState);
                telemetry.addData("KP*error: ",scoringPosition.returnVal()[0]);
                telemetry.addData("KI*i: ",scoringPosition.returnVal()[1]);
                telemetry.addData("KD*d: ",scoringPosition.returnVal()[2]);
                telemetry.update();

                hardware.pivot1.setPower(power);
                hardware.pivot2.setPower(power);

                if (!hardware.limit.getState()) {
                    stopState = (System.nanoTime() - startTime) / 1000000;
                }
                else {
                    startTime = System.nanoTime();
                }
            }
            stop();
            eReset();
    }

    public void downPosition(){
        PIDController scoringPosition = new PIDController(1,0,0,0);
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;

        double ticks = 0;

        while((opModeIsActive() && (stopState <= 250))){
            double position = getPosition();
            double power = scoringPosition.power(ticks,position);

            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("KP*error: ",scoringPosition.returnVal()[0]);
            telemetry.addData("KI*i: ",scoringPosition.returnVal()[1]);
            telemetry.addData("KD*d: ",scoringPosition.returnVal()[2]);
            telemetry.update();

            hardware.pivot1.setPower(power);
            hardware.pivot2.setPower(power);

            if (Math.abs(ticks-position)<= ENCODER_TOLERANCE) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
        }
        stop();

    }

    public void eReset(){
        for(SpeedControlledMotor motor: hardware.pivotMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void stop(){
        for(SpeedControlledMotor motor: hardware.pivotMotors) {
            motor.setPower(0);
        }
    }

    public void setLiftPosition (double liftPosition) {
        this.liftPosition = liftPosition;
    }

    public double getPosition() {
        return hardware.pivot1.getCurrentPosition();
    }

    public void setPower(double power){
        for(SpeedControlledMotor motor: hardware.pivotMotors) {
            motor.setPower(power);
        }
    }

    public double getRawPower () {
        return hardware.pivotMotors[0].getPower();
    }

    public double getAngle(){
        double angle = PIVOT_START_ANGLE + hardware.pivot1.getAngle(PIVOT_TICKS_PER_ROTATION);
        return angle;
    }

    public boolean opModeIsActive() {
        return auto.getOpModeIsActive();
    }

}
