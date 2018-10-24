package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.PathFollower;


@Autonomous(name = "BlueBottomAuto")
public class BlueBottomAuto extends LinearOpMode implements AutonomousOpMode{

    PathFollower trackingNigga = new PathFollower(this);

    @Override
    public boolean getOpModeIsActive() {
        return opModeIsActive();
    }

    public void runOpMode() {

    }
}