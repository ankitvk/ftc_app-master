package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.PathFollower;

@Disabled
@Autonomous(name = "PivotTest")
public class PivotTest extends LinearOpMode implements AutonomousOpMode {

    private Hardware robot = new Hardware();

    public boolean getOpModeIsActive() {
        return opModeIsActive();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    @Override
    public void runOpMode() {

        robot.setAuto(this, telemetry);
        robot.init(hardwareMap);

        robot.getIndex().setPosition(.95);

        waitForStart();

        robot.getPivot().scoringPosition();

        robot.getIndex().setPosition(.15);

        //robot.intake.setPower(-.75);

        sleep(2000);

        //robot.intake.setPower(0);

        robot.getIndex().setPosition(.95);

        robot.getPivot().downPosition();
    }
}
