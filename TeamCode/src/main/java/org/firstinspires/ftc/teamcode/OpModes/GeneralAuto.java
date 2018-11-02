package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.PathFollower;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;


@Autonomous(name = "GeneralAuto")
public class GeneralAuto extends LinearOpMode implements AutonomousOpMode,Constants {

    Hardware robot = new Hardware();
    PathFollower track = new PathFollower(this);
    GoldFind goldfish = new GoldFind(this);
    Drivetrain drivetrain = new Drivetrain(robot);

    @Override
    public boolean getOpModeIsActive() {
        return opModeIsActive();
    }

    public void runOpMode() {
        goldfish.startOpenCV(hardwareMap);
        robot.stopRotation.setPosition(.5);
        drivetrain.rotateToAngle(-30);
        sleep(500);
        while(getOpModeIsActive() && !goldfish.getAligned()){
            drivetrain.rotate(.125,false);
        }
        drivetrain.stop();
        goldfish.disable();

        drivetrain.driveForwardDistance(.5,30);

        if(robot.imu.getYaw()<-25){

        }
        else if(robot.imu.getYaw()>-5 && robot.imu.getYaw()<5){

        }
        else if(robot.imu.getYaw()>25){

        }

    }//end opMode

    private void extend(){
        PIDController control = new PIDController(extensionKP,extensionKI,extensionKD,extensionMaxI);
        double error = Math.abs((robot.extensionLeft.getCurrentPosition()+(robot.extensionRight.getCurrentPosition()))/2-(NEVEREST20_COUNTS_PER_REV*5.5));
        double p;
        double target;
        double currentLoc;
        while(error>150){
            target = (NEVEREST20_COUNTS_PER_REV*5.5);
            currentLoc = (robot.extensionLeft.getCurrentPosition()+robot.extensionRight.getCurrentPosition())/2;
            p = control.power(target,currentLoc);
            robot.extensionLeft.setPower(p);
            robot.extensionRight.setPower(p);
            error = Math.abs(target-currentLoc);
        }
    }

    private void retract(){
        PIDController control = new PIDController(extensionKP,extensionKI,extensionKD,extensionMaxI);
        double error = Math.abs((robot.extensionLeft.getCurrentPosition()+(robot.extensionRight.getCurrentPosition()))/2);
        double p;
        double currentLoc;
        while(error>150){
            currentLoc = (robot.extensionLeft.getCurrentPosition()+robot.extensionRight.getCurrentPosition())/2;
            p = control.power(0,currentLoc);
            robot.extensionLeft.setPower(p);
            robot.extensionRight.setPower(p);
            error = Math.abs(currentLoc);
        }
    }

    private void oscillate(){
        boolean firstTurn = true;
        while(this.getOpModeIsActive() && !goldfish.getAligned()){
            if (firstTurn){
                //rotate 30 degrees
                firstTurn = false;
            }
            //rotate -30 degrees
            sleep(1000);
            //rotate -30 degrees
            sleep(1000);
            //rotate
        }
    }

}