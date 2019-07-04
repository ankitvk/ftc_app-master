package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Autonomous(name = "Opportunity Auto Testing")
public class OpportunityAutoTesting extends LinearOpMode implements AutonomousOpMode, Constants {
    Hardware robot = new Hardware();

    @Override
    public boolean getOpModeIsActive() {
        return opModeIsActive();
    }

    @Override
    public Telemetry getTelemetry() {
        return telemetry;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.setAuto(this,telemetry);
        robot.opportunityInit(hardwareMap);

        waitForStart();

        robot.drivetrain.driveDistance(40,0,0.7,gamepad1);
        //robot.drivetrain.turn(90,gamepad1);
        robot.drivetrain.driveDistance(-30,0,0.8,gamepad1);
    }


}
