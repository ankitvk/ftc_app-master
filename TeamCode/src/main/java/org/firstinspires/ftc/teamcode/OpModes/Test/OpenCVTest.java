package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Prototype.IdentifyGold;

@Autonomous(name = "OpenCVTest",group = "Depot")
public class OpenCVTest extends LinearOpMode implements AutonomousOpMode,Constants {

    Hardware robot = new Hardware();

    public boolean getOpModeIsActive() {
        return opModeIsActive();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    IdentifyGold inspect;

    IdentifyGold.Positions gold;

    @Override
    public void runOpMode() {
        robot.setAuto(this, telemetry);

        robot.init(hardwareMap);

        GoldFind goldfish = new GoldFind(this, robot);
        goldfish.setAlignSettings(ALIGN_POSITION, 1000);
        //start opencv

        waitForStart();

        goldfish.startOpenCV();

        inspect = new IdentifyGold(robot,goldfish);

        while(opModeIsActive()) {
            gold = inspect.identify();
            if (gold == IdentifyGold.Positions.LEFT) {
                telemetry.addLine("LEFT");
            } else if (gold == IdentifyGold.Positions.MIDDLE) {
                telemetry.addLine("MIDDLE");
            } else if (gold == IdentifyGold.Positions.RIGHT) {
                telemetry.addLine("RIGHT");
            }
            telemetry.addData("xPos, ",inspect.returnX());
            telemetry.addData("yPos,",goldfish.getYPosition());
            telemetry.addData("x2",goldfish.getXPosition());
            telemetry.update();
        }

    }
}
