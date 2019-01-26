package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Crater;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.OpModes.Leviathan.Depot.LeviathanDepot;
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

    enum Positions{LEFT,MIDDLE,RIGHT}

    LeviathanCrater.Positions gold;

    @Override
    public void runOpMode() {
        robot.setAuto(this, telemetry);

        robot.init(hardwareMap);

        robot.index.setPosition(.15);
        robot.marker.setPosition(.75);

        goldfish = new GoldFind(this, robot);
        goldfish.setAlignSettings(ALIGN_POSITION, 100);

        goldfish.startOpenCV();

        telemetry.addLine("Instant Run test 3");
        telemetry.update();

        waitForStart();

        if(!goldfish.detector.isFound()){
            gold = LeviathanCrater.Positions.RIGHT;
            telemetry.addLine("RIGHT");
            telemetry.update();
        }
        else if(goldfish.getXPosition()<150){
            gold = LeviathanCrater.Positions.LEFT;
            telemetry.addLine("LEFT");
            telemetry.update();
        }
        else if(goldfish.getXPosition()>=150){
            gold = LeviathanCrater.Positions.MIDDLE;
            telemetry.addLine("MIDDLE");
            telemetry.update();
        }

        robot.endgame.lift();

        robot.drive.driveForwardDistance(-6);

        sleep(2000);

        if(gold == LeviathanCrater.Positions.LEFT){
            robot.craterLeft.run();
        }
        else if(gold == LeviathanCrater.Positions.MIDDLE){
            robot.craterMiddle.run();
        }
        else if(gold == LeviathanCrater.Positions.RIGHT){
            robot.craterRight.run();
        }
    }

}
