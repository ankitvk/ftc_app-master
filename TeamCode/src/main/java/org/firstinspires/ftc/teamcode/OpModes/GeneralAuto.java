package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.PathFollower;


@Autonomous(name = "BlueBottomAuto")
public class GeneralAuto extends LinearOpMode implements AutonomousOpMode,Constants {

    Hardware robot = new Hardware();
    PathFollower track = new PathFollower(this);

    @Override
    public boolean getOpModeIsActive() {
        return opModeIsActive();
    }

    public void runOpMode() {

    }//end opMode

    private void extend(){
        PIDController control = new PIDController(extensionKP,extensionKI,extensionKD,extensionMaxI);
        double error = Math.abs((robot.extensionLeft.getCurrentPosition()+(robot.extensionRight.getCurrentPosition()))/2-(NEVEREST20_COUNTS_PER_REV*5.5));
        double p;
        while(error>150){
            p = control.power((NEVEREST20_COUNTS_PER_REV*5.5),((robot.extensionLeft.getCurrentPosition()+robot.extensionRight.getCurrentPosition())/2));
            robot.extensionLeft.setPower(p);
            robot.extensionRight.setPower(p);
        }
    }

    private void retract(){
        PIDController control = new PIDController(extensionKP,extensionKI,extensionKD,extensionMaxI);
        double error = Math.abs((robot.extensionLeft.getCurrentPosition()+(robot.extensionRight.getCurrentPosition()))/2);
        double p;
        while(error>150){
            p = control.power(0,((robot.extensionLeft.getCurrentPosition()+robot.extensionRight.getCurrentPosition())/2));
            robot.extensionLeft.setPower(p);
            robot.extensionRight.setPower(p);
        }
    }

}