package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.DummyHardware;

@Disabled
@TeleOp(name="dummyop")
public class DummyWinch extends OpMode implements Constants {

    private DummyHardware robot = new DummyHardware();
    public void init(){
        robot.init(hardwareMap);
    }
    public void loop(){
        if(gamepad1.a){
            robot.winch.setPower(1);
        }
        else if(gamepad1.b){
            robot.winch.setPower(-1);
        }
        else{
            robot.winch.setPower(0);
        }
    }
}
