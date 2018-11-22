package org.firstinspires.ftc.teamcode.OpModes.Depot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;
import org.firstinspires.ftc.teamcode.Subsystems.PathFollower;

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

        Drivetrain drivetrain = new Drivetrain(robot);
        PathFollower track = new PathFollower(this);
        robot.init(hardwareMap);
        //start opencv

        waitForStart();

        track.trackPoint(robot,robot.imu,0,10);
        track.trackPoint(robot,robot.imu,10,20);

    drivetrain.stop();
    }//end opMode
}
