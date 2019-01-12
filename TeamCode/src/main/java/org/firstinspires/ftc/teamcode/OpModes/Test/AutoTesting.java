package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Drivetrain;

@Disabled
@Autonomous(name = "AutoTest",group = "Test")
public class AutoTesting extends LinearOpMode implements AutonomousOpMode,Constants {

    Hardware robot = new Hardware();

    public boolean getOpModeIsActive() {
        return opModeIsActive();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    @Override
    public void runOpMode(){
        robot.setAuto(this, telemetry);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.init(hardwareMap);
        double goldPos = 0;

        waitForStart();
        //robot.led.setPosition(-.41);

        drivetrain.driveForwardDistance(20);
        drivetrain.rotateToAbsoluteAngle(-45);
        drivetrain.driveForwardDistance(-45);
        drivetrain.rotateToBigAbsoluteAngle(90);
    }
}
