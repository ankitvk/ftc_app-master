package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
@Disabled
@Autonomous(name = "JustZoom",group = "Dummy")
public class JustZoom extends LinearOpMode implements AutonomousOpMode,Constants{

    Hardware robot = new Hardware();

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

        telemetry.addLine("Instant Run test 3");
        telemetry.update();

        waitForStart();

        robot.drive.driveForwardDistance(45);

        robot.drive.stop();
    }
}
