package org.firstinspires.ftc.teamcode.Subsystems.Paths.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Control.SpeedControlledMotor;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Autonomous(name = "Mineral Time")
public class MineralTime extends LinearOpMode implements AutonomousOpMode,Constants {

    Hardware robot = new Hardware();

    public boolean getOpModeIsActive() {
        return opModeIsActive();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public MineralTime(Hardware hardware){
        this.robot = hardware;
    }

    public void mineralGang(){
        extend();
        down();
        telemetry.addLine("sleepTigor");
        telemetry.update();
        sleep(2000);
        up();
    }

    @Override
    public void runOpMode(){

        robot.setAuto(this, telemetry);

        robot.init(hardwareMap);

        telemetry.addLine("b o x");
        telemetry.update();

        waitForStart();

        mineralGang();
    }

    private void extend(){
        PIDController controlExtend = new PIDController(0.2 ,0,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        extendReset();
        pivotReset();

        while(opModeIsActive() && (stopState <= 125/4)){
            double position = robot.extendo.getCurrentPosition();
            double power = controlExtend.power(-2950,position);

            telemetry.addLine("extend");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("extendo: ", robot.extendo.getCurrentPosition());
            telemetry.addLine(" ");
            telemetry.addData("error: ",controlExtend.getError());
            telemetry.addData("KP*error: ",controlExtend.returnVal()[0]);
            telemetry.addData("KI*i: ",controlExtend.returnVal()[1]);
            telemetry.update();
            robot.extendo.setPower(power);

            if (Math.abs(position+2950) <= 25) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
        }
    }

    private void down(){
        PIDController controlPivot = new PIDController(0.001 ,0,0,0); //increase Ki .00005
        PIDController extendControl = new PIDController(0.001,0,0,1);

        double targetPosition = robot.extendo.getCurrentPosition();


        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125/4)){
            double position = robot.pivot1.getCurrentPosition();
            double power = controlPivot.power(-1000,position);

            telemetry.addLine("down");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("pivot: ", robot.pivot1.getCurrentPosition());
            telemetry.addLine(" ");
            telemetry.addData("error: ",controlPivot.getError());
            telemetry.addData("KP*error: ",controlPivot.returnVal()[0]);
            telemetry.addData("KI*i: ",controlPivot.returnVal()[1]);
            telemetry.update();
            robot.pivot1.setPower(power);
            robot.pivot2.setPower(power);

            double currentPosition = robot.extendo.getCurrentPosition();

            robot.extendo.setPower(-extendControl.power(currentPosition, targetPosition));

            if (Math.abs(position+1000) <= 25) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }

            if(System.nanoTime()/1000000-beginTime/1000000>1000){
                break;
            }
        }
    }

    private void up(){
        PIDController controlPivot = new PIDController(0.03 ,0,0,0); //increase Ki .00005
        PIDController controlExtend = new PIDController(0.2 ,0,0,0); //increase Ki .00005

        long startTime = System.nanoTime();
        long beginTime = startTime;
        long pivotStopState = 0;
        long extendoStopState = 0;
        pivotReset();
        while(opModeIsActive() && (pivotStopState <= 125/4) && (extendoStopState <= 125/4)){
            double pivotPosition = robot.pivot1.getCurrentPosition();
            double pivotPower = controlPivot.power(2700,pivotPosition);

            telemetry.addLine("up");
            telemetry.addData("power", pivotPower);
            telemetry.addData("stopstate: ", pivotStopState);
            telemetry.addData("pivot: ", robot.pivot1.getCurrentPosition());
            telemetry.addLine(" ");
            telemetry.addData("error: ",controlPivot.getError());
            telemetry.addData("KP*error: ",controlPivot.returnVal()[0]);
            telemetry.addData("KI*i: ",controlPivot.returnVal()[1]);
            telemetry.update();
            robot.pivot1.setPower(pivotPower);
            robot.pivot2.setPower(pivotPower);

            if (Math.abs(pivotPosition-2700) <= 15) {
                pivotStopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }

            double extendoPosition = robot.extendo.getCurrentPosition();
            double extendoPower = controlExtend.power(-2450,extendoPosition);

            telemetry.addLine("extend");
            telemetry.addData("power", extendoPower);
            telemetry.addData("stopstate: ", extendoStopState);
            telemetry.addData("extendo: ", robot.extendo.getCurrentPosition());
            telemetry.addLine(" ");
            telemetry.addData("error: ",controlExtend.getError());
            telemetry.addData("KP*error: ",controlExtend.returnVal()[0]);
            telemetry.addData("KI*i: ",controlExtend.returnVal()[1]);
            telemetry.update();
            robot.extendo.setPower(extendoPower);

            if (Math.abs(extendoPosition+2450) <= 25) {
                extendoStopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
        }
    }

    private void extendReset() {
         robot.extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         robot.extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void pivotReset(){
        robot.pivot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.pivot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
}
