package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
@Disabled
@TeleOp( name = "Drive Distance Opportunity Test", group = "Testing")
public class OpportunityAuto extends LinearOpMode implements AutonomousOpMode, Constants {
    private Hardware robot = new Hardware();
    private boolean set = false;
    //private double kp = .029, ki = 0, kd = 0.00003;

    public boolean getOpModeIsActive() {
        return opModeIsActive();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.setAuto(this,telemetry);
        robot.opportunityInit(hardwareMap);
        /*robot.drivetrain.active = opModeIsActive();
        robot.drivetrain.telemetry = telemetry;*/

        waitForStart();

        while(opModeIsActive()){

/*            if(gamepad1.left_bumper){ kp -= .0005;
                telemetry.addData("Kp: ",kp);
                telemetry.addData("Kd: ",kd);
                telemetry.addData("set?: ",set);
                telemetry.update();
            }
            else if(gamepad1.right_bumper) {kp += .0005;
                telemetry.addData("Kp: ",kp);
                telemetry.addData("Kd: ",kd);
                telemetry.addData("set?: ",set);
                telemetry.update();
            }
            else{
                kp += 0;
                telemetry.addData("Kp: ",kp);
                telemetry.addData("Kd: ",kd);
                telemetry.addData("set?: ",set);
                telemetry.update();
            }

            if(gamepad1.dpad_left) {kd -= .0005;
                telemetry.addData("Kp: ",kp);
                telemetry.addData("Kd: ",kd);
                telemetry.addData("set?: ",set);
                telemetry.update();
            }
            else if(gamepad1.dpad_right) {kd += .0005;
                telemetry.addData("Kp: ",kp);
                telemetry.addData("Kd: ",kd);
                telemetry.addData("set?: ",set);
                telemetry.update();
            }
            else{
                kd += 0;
                telemetry.addData("Kp: ",kp);
                telemetry.addData("Kd: ",kd);
                telemetry.addData("set?: ",set);
                telemetry.update();
            }*/
/*

            if(gamepad1.x){
                kd = 0;
                telemetry.addData("Kp: ",kp);
                telemetry.addData("Kd: ",kd);
                telemetry.addData("set?: ",set);
                telemetry.update();
            }

            if(gamepad1.y){
                pid.setKp(kp);
                pid.setKd(kd);
                set = true;
                telemetry.addData("Kp: ",kp);
                telemetry.addData("Kd: ",kd);
                telemetry.addData("set?: ",set);
                telemetry.update();
            }
*/

            if(gamepad1.a){
            robot.drivetrain.turn(90,gamepad1);
            set = false;
            }
            //if(gamepad1.b) robot.drivetrain.stop();


        }

    }



}
