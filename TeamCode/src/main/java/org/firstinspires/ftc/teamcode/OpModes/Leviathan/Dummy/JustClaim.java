package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Dummy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Disabled
@Autonomous(name = "JustClaim",group = "Dummy")
public class JustClaim extends LinearOpMode implements AutonomousOpMode,Constants {
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

        robot.drive.driveForwardDistance(60);

        robot.intake.setPower(-1);

        sleep(5000);

        robot.intake.setPower(0);

        robot.drive.stop();
    }
}
