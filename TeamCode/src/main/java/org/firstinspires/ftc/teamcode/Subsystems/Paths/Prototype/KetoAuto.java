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

public class KetoAuto implements Constants {

    Hardware robot;
    Telemetry telemetry;
    AutonomousOpMode auto;

    public boolean opModeIsActive() {
        return auto.getOpModeIsActive();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public KetoAuto(Hardware hardware){

        this.robot = hardware;
        this.telemetry = hardware.getTelemetry();
        this.auto = hardware.getAuto();

    }

    public void ketonomous(){
        firstTurn();
        goForward();
        secondTurn();
        driveToDepot();
        /*marker();
        driveFromDepot();*/
        /*thirdTurn();
        goBack();
        fourthTurn();*/
    }

    private void marker(){
        robot.getWinch().setPower(-1);
        sleep(2000);
        robot.getWinch().setPower(0);
    }

   /* @Override
    public void runOpMode(){

        robot.setAuto(this, telemetry);

        robot.init(hardwareMap);

        telemetry.addLine("b o x");
        telemetry.update();

        waitForStart();

        ketonomous();

    }*/

    private void  firstTurn(){
        PIDController controlRotate = new PIDController(0.027 ,0,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125/4)){
            double position = robot.getImu().getRelativeYaw();
            double power = controlRotate.power(87.5,position);

            telemetry.addLine("first Turn");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", robot.getImu().getRelativeYaw());
            telemetry.addLine(" ");
            telemetry.addData("error: ",controlRotate.getError());
            telemetry.addData("KP*error: ",controlRotate.returnVal()[0]);
            telemetry.addData("KI*i: ",controlRotate.returnVal()[1]);
            telemetry.update();
            robot.getFrontLeft().setPower(power*.165);
            robot.getBackLeft().setPower(power*.165);
            robot.getFrontRight().setPower(power);
            robot.getBackRight().setPower(power);

            if (Math.abs(position-87.5) <= 1) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
            /*if(System.nanoTime()/1000000-beginTime/1000000>1250){
                break;
            }*/
        }
    }

    private void goForward(){
        eReset();
        PIDController controlDistance = new PIDController(.000490,0,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125/4)){
            double position = robot.getFrontLeft().getCurrentPosition();
            double power = controlDistance.power(distanceToTicks(20),position);

            if(ticksToDistance(position)>19){
                power = 1;
            }

            telemetry.addLine("goForward");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("error: ",controlDistance.getError());
            telemetry.addData("KP*error: ",controlDistance.returnVal()[0]);
            telemetry.addData("KI*i: ", controlDistance.returnVal()[1]);
            telemetry.update();
            robot.getFrontLeft().setPower(-power);
            robot.getBackLeft().setPower(-power);
            robot.getFrontRight().setPower(power);
            robot.getBackRight().setPower(power);

            if (ticksToDistance(Math.abs(position-distanceToTicks(20))) <= 1) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }

            /*if(System.nanoTime()/1000000-beginTime/1000000>2500){
                break;
            }*/

        }
    }

    private void secondTurn(){

        PIDController controlRotate = new PIDController(0.01990,0.2,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125)){
            double position = robot.getImu().getRelativeYaw();
            double power = controlRotate.power(135,position);

            telemetry.addLine("secondTurn");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", robot.getImu().getRelativeYaw());
            telemetry.addLine(" ");
            telemetry.addData("error: ",controlRotate.getError());
            telemetry.addData("KP*error: ",controlRotate.returnVal()[0]);
            telemetry.addData("KI*i: ",controlRotate.returnVal()[1]);
            telemetry.update();
            robot.getFrontLeft().setPower(power*.175);
            robot.getBackLeft().setPower(power*.175);
            robot.getFrontRight().setPower(power);
            robot.getBackRight().setPower(power);

            if (Math.abs(position-135) <= 1) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
            /*if(System.nanoTime()/1000000-beginTime/1000000>1500){
                break;
            }*/
        }
    }

    private void driveToDepot(){
        eReset();
        PIDController controlDistance = new PIDController(.00065,0,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125/4)){
            double position = robot.getFrontLeft().getCurrentPosition();
            double power = controlDistance.power(distanceToTicks(25),position);

            telemetry.addLine("driveToDepot");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", robot.getImu().getRelativeYaw());
            telemetry.addData("error: ",controlDistance.getError());
            telemetry.addData("KP*error: ",controlDistance.returnVal()[0]);
            telemetry.addData("KI*i: ", controlDistance.returnVal()[1]);
            telemetry.update();
            robot.getFrontLeft().setPower(-power);
            robot.getBackLeft().setPower(-power);
            robot.getFrontRight().setPower(power);
            robot.getBackRight().setPower(power);

            if (ticksToDistance(Math.abs(position-distanceToTicks(25))) <= Companion.getDISTANCE_TOLERANCE()) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }

            /*if(System.nanoTime()/1000000-beginTime/1000000>1000){
                break;
            }*/

        }
    }

    private void driveFromDepot(){
        eReset();
        PIDController controlDistance = new PIDController(.0005,0,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125/4)){
            double position = robot.getFrontLeft().getCurrentPosition();
            double power = controlDistance.power(distanceToTicks(-50),position);

            telemetry.addLine("driveFromDepot");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", robot.getImu().getRelativeYaw());
            telemetry.addData("error: ",controlDistance.getError());
            telemetry.addData("KP*error: ",controlDistance.returnVal()[0]);
            telemetry.addData("KI*i: ", controlDistance.returnVal()[1]);
            telemetry.update();
            robot.getFrontLeft().setPower(-power);
            robot.getBackLeft().setPower(-power);
            robot.getFrontRight().setPower(power);
            robot.getBackRight().setPower(power);

            if (ticksToDistance(Math.abs(position-distanceToTicks(-50))) <= Companion.getDISTANCE_TOLERANCE()) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }

            /*if(System.nanoTime()/1000000-beginTime/1000000>1000){
                break;
            }*/

        }
    }

    private void thirdTurn(){
        PIDController controlRotate = new PIDController(0.027,0,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125)){
            double position = robot.getImu().getRelativeYaw();
            double power = controlRotate.power(87,position);

            telemetry.addLine("third Turn");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", robot.getImu().getRelativeYaw());
            telemetry.addLine(" ");
            telemetry.addData("error: ",controlRotate.getError());
            telemetry.addData("KP*error: ",controlRotate.returnVal()[0]);
            telemetry.addData("KI*i: ",controlRotate.returnVal()[1]);
            telemetry.update();
            robot.getFrontLeft().setPower(power*.175);
            robot.getBackLeft().setPower(power*.175);
            robot.getFrontRight().setPower(power);
            robot.getBackRight().setPower(power);

            if (Math.abs(position-87) <= 1) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
            /*if(System.nanoTime()/1000000-beginTime/1000000>1500){
                break;
            }*/
        }
    }

    private void goBack(){
        eReset();
        PIDController controlDistance = new PIDController(.000425,0,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125/4)){
            double position = robot.getFrontLeft().getCurrentPosition();
            double power = controlDistance.power(distanceToTicks(-27),position);

            telemetry.addLine("goBack");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("error: ",controlDistance.getError());
            telemetry.addData("KP*error: ",controlDistance.returnVal()[0]);
            telemetry.addData("KI*i: ", controlDistance.returnVal()[1]);
            telemetry.update();
            robot.getFrontLeft().setPower(-power);
            robot.getBackLeft().setPower(-power);
            robot.getFrontRight().setPower(power);
            robot.getBackRight().setPower(power);

            if (ticksToDistance(Math.abs(position-distanceToTicks(-27))) <= 1) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }

            /*if(System.nanoTime()/1000000-beginTime/1000000>2000){
                break;
            }*/

        }
    }

    private void fourthTurn(){
        PIDController controlRotate = new PIDController(0.026,0,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125/4)){
            double position = robot.getImu().getRelativeYaw();
            double power = controlRotate.power(0,position);

            telemetry.addLine("fourth Turn");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", robot.getImu().getRelativeYaw());
            telemetry.addLine(" ");
            telemetry.addData("error: ",controlRotate.getError());
            telemetry.addData("KP*error: ",controlRotate.returnVal()[0]);
            telemetry.addData("KI*i: ",controlRotate.returnVal()[1]);
            telemetry.update();
            robot.getFrontLeft().setPower(power*.165);
            robot.getBackLeft().setPower(power*.165);
            robot.getFrontRight().setPower(power);
            robot.getBackRight().setPower(power);

            if (Math.abs(position-0) <= 1.5) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
            /*if(System.nanoTime()/1000000-beginTime/1000000>1500){
                break;
            }*/
        }
    }

    private double distanceToTicks(double distance){
        return (distance/(Companion.getWHEEL_DIAMETER() *Math.PI))* Companion.getDT_GEARBOX_TICKS_PER_ROTATION();
    }

    private double ticksToDistance(double ticks){
        return (ticks*(Companion.getWHEEL_DIAMETER() *Math.PI))/ Companion.getDT_GEARBOX_TICKS_PER_ROTATION();
    }

    private void eReset() {

        for(SpeedControlledMotor motor: robot.getDrivetrainMotors()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


}
