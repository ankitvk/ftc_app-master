package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Leviathan;

import ftc.library.MaelstromWrappers.MaelstromController;
import ftc.library.MaelstromWrappers.MaelstromOp;

@TeleOp(name = "Tarun's Teleop", group = "Lib_Teleop")
public class LibTeleop_Leviathan extends MaelstromOp implements Constants {
    //public Opportunity robot = new Opportunity();
    public Leviathan robot = new Leviathan();
    @Override
    public void init() {
        //robot.initHardware(hardwareMap);
        robot.initHardware(hardwareMap);
/*        robot.pivot.reset();

        feed.add("Limit: ",robot.limit.getSignalValue());
        feed.add("Pivot Angle: ",robot.pivot.getAngle());
        feed.update();*/
    }

    @Override
    public void loop() {
        //robot.driveTeleop(controller1);
        robot.driveTeleop(controller1);
        robot.intake.DriverControl(controller1, MaelstromController.Toggler.OFF);
        robot.lift.DriverControl(controller1);
        robot.pivot.DriverControl(controller2);
        robot.pivot.setLiftPosition(robot.lift.getCounts());
        robot.index(controller2);
        if(controller2.rightTriggerPressed()) robot.lift();
        else if(controller2.leftTriggerPressed()) robot.lower();
    }
}
