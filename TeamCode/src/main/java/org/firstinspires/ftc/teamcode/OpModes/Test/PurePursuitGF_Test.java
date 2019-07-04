package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.RobotMovement;

@Autonomous(name = "GF Pure Pursuit Test")
public class PurePursuitGF_Test extends LinearOpMode implements Constants, AutonomousOpMode {

    public Hardware robot =  new Hardware();
    private RobotMovement movement;

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
        robot.opportunityInit(hardwareMap);
        robot.setAuto(this,telemetry);
        robot.drivetrain.eReset();
        movement = new RobotMovement(robot);

        waitForStart();

        while(opModeIsActive()){
            movement.goToPosition(0,75,1);
        }
    }


}
