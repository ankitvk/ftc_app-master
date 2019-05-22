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

@TeleOp(name = "One Man Gang")
public class OneManGangTeleop extends OpMode implements Constants {

    private Hardware robot = new Hardware();

    private Pivot pivot = new Pivot(robot);
    private Extendo extendo = new Extendo(robot);
    private Intake intake =  new Intake(robot);
    private Drivetrain drivetrain = new Drivetrain(robot);
    private Endgame endgame = new Endgame(robot);

    private boolean resetimu = true;


    public void init(){
        robot.init(hardwareMap);
        /*robot.hookRelease.setPosition(0);
        //robot.led.setPosition(-.41);
        robot.hookSwivel.setPosition(.75);
        robot.led.setPattern(PATTERN);*/

    }
    public void loop(){

        drivetrain.drive(gamepad1);
        pivot.driverControlSingle(gamepad1);
        extendo.driverControl(gamepad1);
        intake.intake(gamepad1);
        intake.index(gamepad1,true);

        //telemetry.addData("Angle: ",robot.imu.getRelativeYaw());
        telemetry.addData("State: ", robot.getLimit().getState());
        telemetry.update();

    }
}