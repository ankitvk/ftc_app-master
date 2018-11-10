package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;

@Autonomous(name = "CraterAuto")
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

        GoldFind goldfish = new GoldFind(this);
        goldfish.setAlignSettings(ALIGN_POSITION, 1000);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.init(hardwareMap);
        goldfish.startOpenCV(hardwareMap); //start opencv
        double goldPos = 0;

        waitForStart();

        drivetrain.rotateForTime(-.5, 500);

        drivetrain.stop();

        /*drivetrain.rotate(-1);
        sleep(125);*/
        while (getOpModeIsActive() && !goldfish.getAligned()) {
            drivetrain.rotate(-0.3);
            telemetry.addData("Aligned:", goldfish.getAligned());
            telemetry.addData("Pos:", goldfish.getXPosition());
            telemetry.update();
        }
        drivetrain.stop();
        goldfish.disable();

        drivetrain.driveForwardDistance(-30);
    }
}