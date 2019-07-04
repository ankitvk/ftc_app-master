package org.firstinspires.ftc.teamcode.OpModes.Undertow.Depot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.PathFollower;

//@Disabled
@Autonomous(name = "Wacky",group = "Depot")
public class TheoreticalDepotAuto extends LinearOpMode implements AutonomousOpMode,Constants {
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
        robot.opportunityInit(hardwareMap);
        PathFollower track = new PathFollower(this,robot,telemetry);
        //start opencv

        waitForStart();

        track.trackPoint(0,5);

        //track.trackPoint(10,20);

        robot.drivetrain.stop();
    }//end opMode
}
