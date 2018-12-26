package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

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
    private double basePower = 0.7;
    private double baseDownPower = 0.5;
    private double rotatorPower = basePower;
    private double downPower = -0.1;

    public PIDController pivotControl = new PIDController(kp,ki,kd,1);

    private double liftPosition = 0;

    Hardware hardware;

    public Pivot(Hardware hardware){
        this.hardware=hardware;
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
        } else {
            setPower(0);
        }

        if (controller.right_bumper|| controller.left_bumper) targetPosition = getPosition();
        else {
            double currentPosition = getPosition();
            //setPower(pivotControl.power(targetPosition,currentPosition));
        }
        pivotControl.setKp(kp);
        pivotControl.setKi(ki);
        pivotControl.setKd(kd);
    }

    private void eReset(){
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
        return (hardware.pivotMotors[0].getCurrentPosition());
    }

    public void setPower(double power){
        for(SpeedControlledMotor motor: hardware.pivotMotors) {
            motor.setPower(power);
        }
    }

    public double getRawPower () {
        return hardware.pivotMotors[0].getPower();
    }

    private double getAngle(){
        double angle = 0;
        for(SpeedControlledMotor motor: hardware.pivotMotors) {
            angle += motor.getAngle(PIVOT_TICKS_PER_INCH,PIVOT_TICKS_PER_ROTATION);
        }
        return angle;
    }

}
