package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.PathFollower;

@Autonomous(name = "PathFollow",group = "Test")
public class PathFollowTest extends LinearOpMode implements AutonomousOpMode,Constants {

    Hardware robot = new Hardware();
    PathFollower pathFollower = new PathFollower(this,robot);

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

        waitForStart();

        pathFollower.trackPoint(5,5);

    }
}
