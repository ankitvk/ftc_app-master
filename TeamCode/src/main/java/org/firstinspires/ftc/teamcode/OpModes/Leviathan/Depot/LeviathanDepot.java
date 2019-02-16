package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Depot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;

@Autonomous(name = "LeviathanDepot",group = "Depot")
public class LeviathanDepot extends LinearOpMode implements AutonomousOpMode,Constants {
    Hardware robot = new Hardware();
    private GoldFind goldfish;


    public boolean getOpModeIsActive() {
        return opModeIsActive();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    enum Positions{LEFT,MIDDLE,RIGHT}

    Positions gold;

    @Override
    public void runOpMode() {
        robot.setAuto(this, telemetry);

        robot.init(hardwareMap);

        robot.index.setPosition(.15);

        goldfish = new GoldFind(this, robot);
        goldfish.setAlignSettings(ALIGN_POSITION, 100);

        goldfish.startOpenCV();

        telemetry.addLine("Instant Run test 3");
        telemetry.update();

        waitForStart();

        /*if(!goldfish.detector.isFound()){
            gold = Positions.RIGHT;
            telemetry.addLine("RIGHT");
            telemetry.update();
        }
        else if(goldfish.getXPosition()<150){
            gold = Positions.LEFT;
            telemetry.addLine("LEFT");
            telemetry.update();
        }
        else if(goldfish.getXPosition()>=150){
            gold = Positions.MIDDLE;
            telemetry.addLine("MIDDLE");
            telemetry.update();
        }

        robot.endgame.lift();

        robot.drive.driveForwardDistance(-6);

        robot.winch.setPower(1);

        sleep(5000);

        robot.winch.setPower(0);*/

        gold = Positions.RIGHT;

        if(gold == Positions.LEFT){
            robot.depotLeft.run();
        }
        else if(gold == Positions.MIDDLE){
            robot.depotMiddle.run();
        }
        else if(gold == Positions.RIGHT){
            robot.depotRight.run();
        }
    }

}
