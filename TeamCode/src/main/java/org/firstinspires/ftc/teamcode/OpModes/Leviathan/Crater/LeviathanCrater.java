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

        robot.getIndex().setPosition(.15);

        robot.getHangLeftRelease().setPosition(.3);
        robot.getHangRightRelease().setPosition(.85);

        goldfish = new GoldFind(this, robot);
        goldfish.setAlignSettings(Companion.getALIGN_POSITION(), 100);

        goldfish.startOpenCV();

        inspect = new IdentifyGold(robot,goldfish);

        telemetry.addLine("Icey");
        telemetry.update();

        waitForStart();

        gold = inspect.identify();

        robot.getHang().drop();

        if(gold == IdentifyGold.Positions.LEFT){
            robot.getCraterLeft().run();
        }
        else if(gold ==  IdentifyGold.Positions.MIDDLE){
            robot.getCraterMiddle().run();
        }
        else if(gold ==  IdentifyGold.Positions.RIGHT){
            robot.getCraterRight().run();
        }

        robot.getExtendo().extend(1);
        sleep(4000);
        robot.getExtendo().stop();

    }

}
