package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Crater;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;

@Autonomous(name = "LeviathanCrater",group = "Crater")
public class LeviathanCrater extends LinearOpMode implements AutonomousOpMode,Constants {
    Hardware robot = new Hardware();
    private GoldFind goldfish;


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

        goldfish = new GoldFind(this, robot);
        goldfish.setAlignSettings(ALIGN_POSITION, 1000);

        robot.hookRelease.setPosition(0);
        telemetry.addLine("Instant Run test 3");
        telemetry.update();


        waitForStart();

        goldfish.startOpenCV();

        robot.hookRelease.setPosition(0);

        sleep(500);

        robot.drop.setPosition(1);

        sleep(1000);

        robot.hookSwivel.setPosition(.75);

        robot.drive.rotateToAbsoluteAngle(60); //Doesn't have to be so complicated

        while(getOpModeIsActive()&& !goldfish.isFound()){
            robot.drive.rotate(-.50);
            telemetry.addData("Found: ",goldfish.isFound());
            telemetry.addData("XPos: ",goldfish.getXPosition());
            telemetry.update();
        }

        goldfish.alignGold();

        goldfish.disable();

        robot.drive.driveForwardDistance(40);

        robot.drive.stop();
    }

}
