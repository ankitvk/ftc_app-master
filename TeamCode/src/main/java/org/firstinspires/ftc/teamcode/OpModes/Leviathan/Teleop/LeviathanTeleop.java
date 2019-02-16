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

    }
    public void loop(){

        drivetrain.drive(gamepad1);
        pivot.driverControl(gamepad2);
        extendo.driverControl(gamepad1);
        intake.intake(gamepad1);
        intake.index(gamepad2);

        telemetry.addData("pivot1",robot.pivot1.getCurrentPosition());
        telemetry.addData("pivot2",robot.pivot2.getCurrentPosition());
        telemetry.addData("winch?",robot.winch.getCurrentPosition());
        telemetry.addData("extendo",robot.extendo.getCurrentPosition());

        //robot.ledRiver.setColor(Color.BLUE);
        //endgame.winch(gamepad2);

        /*telemetry.addData("backRightRPM:",robot.backRight.getRPM());
        telemetry.addData("frontRightRPM:",robot.frontRight.getRPM());
        telemetry.addData("backLeftRPM:",robot.backLeft.getRPM());
        telemetry.addData("frontLeftRPM:",robot.frontLeft.getRPM());
        telemetry.addData("backRightPower:",robot.backRight.getPower());
        telemetry.addData("frontRightPower:",robot.frontRight.getPower());
        telemetry.addData("backLeftPower:",robot.backLeft.getPower());
        telemetry.addData("frontLeftPower:",robot.frontLeft.getPower());
*/
        //telemetry.addData("limit",robot.limit.getState());
        telemetry.update();

    }
}