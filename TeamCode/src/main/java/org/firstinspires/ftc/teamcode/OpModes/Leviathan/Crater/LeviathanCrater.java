package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Crater;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.OpModes.Leviathan.Depot.LeviathanDepot;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Prototype.IdentifyGold;

@Autonomous(name = "LeviathanCrater",group = "Crater")
public class LeviathanCrater extends LinearOpMode implements AutonomousOpMode,Constants {

    Hardware robot = new Hardware();
    IdentifyGold inspect;
    private GoldFind goldfish;


    public boolean getOpModeIsActive() {
        return opModeIsActive();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    IdentifyGold.Positions gold;

    @Override
    public void runOpMode() {
        robot.setAuto(this, telemetry);

        robot.init(hardwareMap);

        robot.index.setPosition(.15);

        robot.hangLeftRelease.setPosition(.3);
        robot.hangRightRelease.setPosition(.85);

        goldfish = new GoldFind(this, robot);
        goldfish.setAlignSettings(ALIGN_POSITION, 100);

        goldfish.startOpenCV();

        inspect = new IdentifyGold(robot,goldfish);

        telemetry.addLine("Icey");
        telemetry.update();

        waitForStart();

        gold = inspect.identify();

        robot.hang.drop();

        if(gold == IdentifyGold.Positions.LEFT){
            robot.craterLeft.run();
        }
        else if(gold ==  IdentifyGold.Positions.MIDDLE){
            robot.craterMiddle.run();
        }
        else if(gold ==  IdentifyGold.Positions.RIGHT){
            robot.craterRight.run();
        }

        robot.extendo.extend(1);
        sleep(4000);
        robot.extendo.stop();

    }

}
