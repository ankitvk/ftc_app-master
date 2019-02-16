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

@Autonomous(name = "Keto Auto",group = "Dummy")
public class KetoAuto extends LinearOpMode implements AutonomousOpMode,Constants {

    Hardware robot = new Hardware();

    public boolean getOpModeIsActive() {
        return opModeIsActive();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    @Override
    public void runOpMode(){

        robot.setAuto(this, telemetry);

        robot.init(hardwareMap);

        telemetry.addLine("b o x");
        telemetry.update();

        waitForStart();

        firstTurn();
        goForward();
        secondTurn();
        //driveToDepot();
        sleep(3000);
        //driveFromDepot();
        thirdTurn();
        goBack();
        fourthTurn();
    }

    private void firstTurn(){
        PIDController controlRotate = new PIDController(0.02175 ,1,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125/4)){
            double position = robot.imu.getRelativeYaw();
            double power = controlRotate.power(87.5,position);

            telemetry.addLine("first Turn");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", robot.imu.getRelativeYaw());
            telemetry.addLine(" ");
            telemetry.addData("error: ",controlRotate.getError());
            telemetry.addData("KP*error: ",controlRotate.returnVal()[0]);
            telemetry.addData("KI*i: ",controlRotate.returnVal()[1]);
            telemetry.update();
            robot.frontLeft.setPower(power*.165);
            robot.backLeft.setPower(power*.165);
            robot.frontRight.setPower(power);
            robot.backRight.setPower(power);

            if (Math.abs(position-87.5) <= 1) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
            if(System.nanoTime()/1000000-beginTime/1000000>1250){
                break;
            }
        }
    }

    private void goForward(){
        eReset();
        PIDController controlDistance = new PIDController(.000385,0,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125/4)){
            double position = robot.frontLeft.getCurrentPosition();
            double power = controlDistance.power(distanceToTicks(25),position);

            if(ticksToDistance(position)>23){
                power = 1;
            }

            telemetry.addLine("goForward");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("error: ",controlDistance.getError());
            telemetry.addData("KP*error: ",controlDistance.returnVal()[0]);
            telemetry.addData("KI*i: ", controlDistance.returnVal()[1]);
            telemetry.update();
            robot.frontLeft.setPower(-power);
            robot.backLeft.setPower(-power);
            robot.frontRight.setPower(power);
            robot.backRight.setPower(power);

            if (ticksToDistance(Math.abs(position-distanceToTicks(25))) <= 1) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }

            if(System.nanoTime()/1000000-beginTime/1000000>2500){
                break;
            }

        }
    }

    private void secondTurn(){

        PIDController controlRotate = new PIDController(0.01775,0.2,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125)){
            double position = robot.imu.getRelativeYaw();
            double power = controlRotate.power(135,position);

            telemetry.addLine("secondTurn");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", robot.imu.getRelativeYaw());
            telemetry.addLine(" ");
            telemetry.addData("error: ",controlRotate.getError());
            telemetry.addData("KP*error: ",controlRotate.returnVal()[0]);
            telemetry.addData("KI*i: ",controlRotate.returnVal()[1]);
            telemetry.update();
            robot.frontLeft.setPower(power*.175);
            robot.backLeft.setPower(power*.175);
            robot.frontRight.setPower(power);
            robot.backRight.setPower(power);

            if (Math.abs(position-135) <= 1) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
            if(System.nanoTime()/1000000-beginTime/1000000>1500){
                break;
            }
        }
    }

    private void driveToDepot(){
        eReset();
        PIDController controlDistance = new PIDController(.001,0,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125/4)){
            double position = robot.frontLeft.getCurrentPosition();
            double power = controlDistance.power(distanceToTicks(10),position);

            telemetry.addLine("driveToDepot");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", robot.imu.getRelativeYaw());
            telemetry.addData("error: ",controlDistance.getError());
            telemetry.addData("KP*error: ",controlDistance.returnVal()[0]);
            telemetry.addData("KI*i: ", controlDistance.returnVal()[1]);
            telemetry.update();
            robot.frontLeft.setPower(-power);
            robot.backLeft.setPower(-power);
            robot.frontRight.setPower(power);
            robot.backRight.setPower(power);

            if (ticksToDistance(Math.abs(position-distanceToTicks(10))) <= DISTANCE_TOLERANCE) {
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
        PIDController controlDistance = new PIDController(.001,0,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125/4)){
            double position = robot.frontLeft.getCurrentPosition();
            double power = controlDistance.power(distanceToTicks(-12.5),position);

            telemetry.addLine("driveFromDepot");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", robot.imu.getRelativeYaw());
            telemetry.addData("error: ",controlDistance.getError());
            telemetry.addData("KP*error: ",controlDistance.returnVal()[0]);
            telemetry.addData("KI*i: ", controlDistance.returnVal()[1]);
            telemetry.update();
            robot.frontLeft.setPower(-power);
            robot.backLeft.setPower(-power);
            robot.frontRight.setPower(power);
            robot.backRight.setPower(power);

            if (ticksToDistance(Math.abs(position-distanceToTicks(-12.5))) <= DISTANCE_TOLERANCE) {
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
        PIDController controlRotate = new PIDController(0.02325,0,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125)){
            double position = robot.imu.getRelativeYaw();
            double power = controlRotate.power(92.5,position);

            telemetry.addLine("third Turn");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", robot.imu.getRelativeYaw());
            telemetry.addLine(" ");
            telemetry.addData("error: ",controlRotate.getError());
            telemetry.addData("KP*error: ",controlRotate.returnVal()[0]);
            telemetry.addData("KI*i: ",controlRotate.returnVal()[1]);
            telemetry.update();
            robot.frontLeft.setPower(power*.175);
            robot.backLeft.setPower(power*.175);
            robot.frontRight.setPower(power);
            robot.backRight.setPower(power);

            if (Math.abs(position-92.5) <= 1) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
            if(System.nanoTime()/1000000-beginTime/1000000>1500){
                break;
            }
        }
    }

    private void goBack(){
        eReset();
        PIDController controlDistance = new PIDController(.000425,0,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125/4)){
            double position = robot.frontLeft.getCurrentPosition();
            double power = controlDistance.power(distanceToTicks(-23),position);

            telemetry.addLine("goBack");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("error: ",controlDistance.getError());
            telemetry.addData("KP*error: ",controlDistance.returnVal()[0]);
            telemetry.addData("KI*i: ", controlDistance.returnVal()[1]);
            telemetry.update();
            robot.frontLeft.setPower(-power);
            robot.backLeft.setPower(-power);
            robot.frontRight.setPower(power);
            robot.backRight.setPower(power);

            if (ticksToDistance(Math.abs(position-distanceToTicks(-23))) <= 1) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }

            if(System.nanoTime()/1000000-beginTime/1000000>2000){
                break;
            }

        }
    }

    private void fourthTurn(){
        PIDController controlRotate = new PIDController(0.0245,0,0,0); //increase Ki .00005
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 125/4)){
            double position = robot.imu.getRelativeYaw();
            double power = controlRotate.power(0,position);

            telemetry.addLine("fourth Turn");
            telemetry.addData("power", power);
            telemetry.addData("stopstate: ", stopState);
            telemetry.addData("Angle: ", robot.imu.getRelativeYaw());
            telemetry.addLine(" ");
            telemetry.addData("error: ",controlRotate.getError());
            telemetry.addData("KP*error: ",controlRotate.returnVal()[0]);
            telemetry.addData("KI*i: ",controlRotate.returnVal()[1]);
            telemetry.update();
            robot.frontLeft.setPower(power*.165);
            robot.backLeft.setPower(power*.165);
            robot.frontRight.setPower(power);
            robot.backRight.setPower(power);

            if (Math.abs(position-0) <= 1.5) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
            if(System.nanoTime()/1000000-beginTime/1000000>1500){
                break;
            }
        }
    }

    private double distanceToTicks(double distance){
        return (distance/(WHEEL_DIAMETER*Math.PI))*DT_GEARBOX_TICKS_PER_ROTATION;
    }

    private double ticksToDistance(double ticks){
        return (ticks*(WHEEL_DIAMETER*Math.PI))/DT_GEARBOX_TICKS_PER_ROTATION;
    }

    private void eReset() {

        for(SpeedControlledMotor motor: robot.drivetrainMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
