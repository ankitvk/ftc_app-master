package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Dummy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Autonomous(name = "JustDrop",group = "Dummy")
public class JustDrop extends LinearOpMode implements AutonomousOpMode,Constants{
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

        robot.getEndgame().lift();

        robot.getDrivetrain().driveForwardDistance(-5);
    }
}
