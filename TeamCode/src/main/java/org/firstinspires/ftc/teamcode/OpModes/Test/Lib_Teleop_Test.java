package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Opportunity;

import ftc.library.MaelSubsystems.MaelstromDrivetrain.DrivetrainModels;
import ftc.library.MaelUtils.MaelUtils;
import ftc.library.MaelWrappers.MaelIterative;
import ftc.library.MaelWrappers.MaelLinearOp;

@TeleOp(name = "Library Testing with Opportunity",group = "Testing")
public class Lib_Teleop_Test extends MaelLinearOp implements Constants {
    public Opportunity robot = new Opportunity();
    FtcDashboard dash = MaelUtils.dashboard;
    TelemetryPacket telemetryPacket = new TelemetryPacket();

/*    @Override
    public void initLoop() {
        robot.initHardware(hardwareMap);
        while(!opModeIsActive()){
            displayData();
        }
}

    @Override
    public void teleop() {
        robot.driveTeleop(controller1);
*//*        telemetryPacket.put("FL: ", robot.dt.fl.getPower());
        telemetryPacket.put("FR: ", robot.dt.fr.getPower());
        telemetryPacket.put("BL: ", robot.dt.bl.getPower());
        telemetryPacket.put("BR: ", robot.dt.br.getPower());
        telemetryPacket.put("X: ", robot.tankTracker.getX());
        telemetryPacket.put("Y: ", robot.tankTracker.getY());
        telemetryPacket.put("Theta: ", robot.tankTracker.getHeading());
        dash.sendTelemetryPacket(telemetryPacket);*//*
    }

    @Override*/
    public void displayData() {
        if(!opModeIsActive()){
            feed.add("Is this working?");
            feed.add("Imu: ", robot.imu.getYaw());
        }
        else if(opModeIsActive()) {
            feed.add("FL: ", robot.dt.fl.getPower());
            feed.add("FR: ", robot.dt.fr.getPower());
            feed.add("BL: ", robot.dt.bl.getPower());
            feed.add("BR: ", robot.dt.br.getPower());
            feed.add("X: ", robot.tankTracker.getX());
            feed.add("Y: ", robot.tankTracker.getY());
            feed.add("Theta: ", robot.tankTracker.getHeading());
            //dash.sendTelemetryPacket(telemetryPacket);
        }
    }

    @Override
    public void run() throws InterruptedException {
        robot.initHardware(hardwareMap);
        while(!opModeIsActive()){
            feed.add("Is this working?");
            feed.add("Imu: ", robot.imu.getYaw());
            feed.update();
        }

        waitForStart();

        while(opModeIsActive()){
            robot.driveTeleop(controller1);

            feed.add("FL: ", robot.dt.fl.getPower());
            feed.add("FR: ", robot.dt.fr.getPower());
            feed.add("BL: ", robot.dt.bl.getPower());
            feed.add("BR: ", robot.dt.br.getPower());
            feed.add("X: ", robot.tankTracker.getX());
            feed.add("Y: ", robot.tankTracker.getY());
            feed.add("Theta: ", robot.tankTracker.getHeading());
            feed.update();
        }
    }

    @Override
    public void initHardware() {

    }
}
