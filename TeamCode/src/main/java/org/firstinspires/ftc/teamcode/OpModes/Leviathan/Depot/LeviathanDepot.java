package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Depot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;
import org.firstinspires.ftc.teamcode.Subsystems.Paths.Prototype.IdentifyGold;

@Autonomous(name = "LeviathanDepot",group = "Depot")
public class LeviathanDepot extends LinearOpMode implements AutonomousOpMode,Constants {
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

        goldfish = new GoldFind(this, robot);
        goldfish.setAlignSettings(ALIGN_POSITION, 100);

        goldfish.startOpenCV();

        inspect = new IdentifyGold(robot,goldfish);

        telemetry.addLine("Instant Run test 3");
        telemetry.update();

        waitForStart();

        gold = inspect.identify();

        robot.hang.drop();

        if(gold == IdentifyGold.Positions.LEFT){
            robot.depotLeft.run();
        }
        else if(gold ==  IdentifyGold.Positions.MIDDLE){
            robot.depotMiddle.run();
        }
        else if(gold ==  IdentifyGold.Positions.RIGHT){
            robot.depotRight.run();
        }
    }

}
