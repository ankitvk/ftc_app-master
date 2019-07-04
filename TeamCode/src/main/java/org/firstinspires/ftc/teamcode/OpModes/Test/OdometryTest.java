package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Opportunity;
import org.firstinspires.ftc.teamcode.Subsystems.RobotMovement;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelSubsystems.MaelstromDrivetrain.DrivetrainModels;
import ftc.library.MaelWrappers.MaelIterative;

@TeleOp(name = "Tank Odometry Test")
public class OdometryTest extends MaelIterative /*OpMode*/ implements Constants {
    private Opportunity robot = new Opportunity();
    private MaelPose current;

    @Override
    public void initLoop() {
        robot.initHardware(hardwareMap);
        robot.tankTracker.reset();
        robot.dt.setModel(DrivetrainModels.ARCADE);
        current = robot.tankTracker.toPose();
        while(!opModeIsActive()){
            displayData();
        }

    }

    @Override
    public void teleop() {
       robot.driveTeleop(controller1);
       displayData();
    }

    @Override
    public void displayData(){
        feed.add("X: ", current.x);
        feed.add("Y: ", current.y);
        feed.add("Angle: ", current.angle);
        feed.update();
    }

    @Override
    public void initHardware() {

    }
/*    private Hardware robot = new Hardware();
    private RobotMovement movement;
    @Override
    public void init() {
        robot.opportunityInit(hardwareMap);
        movement = new RobotMovement(robot);
    }

    @Override
    public void loop() {
        robot.drivetrain.drive(gamepad1);

        telemetry.addData("X: ", movement.trackX());
        telemetry.addData("Y: ", movement.trackY());
        telemetry.addData("Angle: ", movement.trackAngle());
        telemetry.update();
    }*/
}
