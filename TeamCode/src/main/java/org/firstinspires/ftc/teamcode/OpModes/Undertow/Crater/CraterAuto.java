package org.firstinspires.ftc.teamcode.OpModes.Undertow.Crater;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;

@Disabled
@Autonomous(name = "CraterAuto",group = "Crater")
public class CraterAuto extends LinearOpMode implements AutonomousOpMode,Constants {

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

        GoldFind goldfish = new GoldFind(this,robot);
        goldfish.setAlignSettings(ALIGN_POSITION, 1000);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.init(hardwareMap);

        double goldPos = 0;

        waitForStart();

        goldfish.startOpenCV(); //start opencv

        /*robot.marker.setPosition(.9);

        robot.hook.setPosition(.5);*/
        robot.extend.setPower(1);
        sleep(1250);
        robot.extend.setPower(0);

        drivetrain.rotateForTime(-.5, 350);
        drivetrain.stop();

        robot.extend.setPower(-1);
        sleep(1500);
        robot.extend.setPower(0);



        while (getOpModeIsActive() && !goldfish.getAligned()) {
            drivetrain.rotate(-0.25);
            telemetry.addData("Aligned:", goldfish.getAligned());
            telemetry.addData("Pos:", goldfish.getXPosition());
            telemetry.update();
        }
        drivetrain.stop();
        goldfish.disable();

        drivetrain.driveForwardDistance(-45);
    }
}