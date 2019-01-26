package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Endgame;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Extendo;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Pivot;

@TeleOp(name="LeviathanTeleop")
public class LeviathanTeleop extends OpMode implements Constants{

    private Hardware robot = new Hardware();

    private Pivot pivot = new Pivot(robot);
    private Extendo extendo = new Extendo(robot);
    private Intake intake =  new Intake(robot);
    private Drivetrain drivetrain = new Drivetrain(robot);
    private Endgame endgame = new Endgame(robot);

    private boolean resetimu = true;


    public void init(){
        robot.init(hardwareMap);
        //robot.hookRelease.setPosition(0);
        //robot.led.setPosition(-.41);
        //robot.hookSwivel.setPosition(.75);
        //robot.led.setPattern(PATTERN);
        robot.marker.setPosition(.15);

    }
    public void loop(){

        drivetrain.drive(gamepad1);
        pivot.driverControl(gamepad2);
        extendo.driverControl(gamepad1);
        intake.intake(gamepad1);
        intake.index(gamepad2);
        endgame.winch(gamepad2);

        /*telemetry.addData("backRightRPM:",robot.backRight.getRPM());
        telemetry.addData("frontRightRPM:",robot.frontRight.getRPM());
        telemetry.addData("backLeftRPM:",robot.backLeft.getRPM());
        telemetry.addData("frontLeftRPM:",robot.frontLeft.getRPM());
        telemetry.addData("backRightPower:",robot.backRight.getPower());
        telemetry.addData("frontRightPower:",robot.frontRight.getPower());
        telemetry.addData("backLeftPower:",robot.backLeft.getPower());
        telemetry.addData("frontLeftPower:",robot.frontLeft.getPower());
*/
        telemetry.addData("pivot1Power:",robot.pivot1.getPower());
        telemetry.addData("pivot2Power:",robot.pivot2.getPower());
        telemetry.addData("extendoPower",robot.extendo.getPower());
        telemetry.addData("pivot1Encoder:",robot.pivot1.getCurrentPosition());
        telemetry.addData("pivot2Encoder:",robot.pivot2.getCurrentPosition());
        telemetry.addData("extendoEncoder",robot.extendo.getCurrentPosition());
        telemetry.addData("limit",robot.limit.getState());
        telemetry.update();

    }
}