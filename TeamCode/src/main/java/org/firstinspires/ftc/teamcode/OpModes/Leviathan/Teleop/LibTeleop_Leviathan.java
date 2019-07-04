package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Leviathan;
import org.firstinspires.ftc.teamcode.Subsystems.Components.MaelPivot;

import ftc.library.MaelSubsystems.MaelCollector;
import ftc.library.MaelSubsystems.MaelElevator;
import ftc.library.MaelWrappers.MaelController;
import ftc.library.MaelWrappers.MaelLinearOp;

@TeleOp(name = "Tarun's Teleop", group = "Lib_Teleop")
public class LibTeleop_Leviathan extends MaelLinearOp implements Constants {
    //public Opportunity robot = new Opportunity();
    public Leviathan robot = new Leviathan();

        //robot.initHardware(hardwareMap);
/*        robot.pivot.reset();

        feed.add("Limit: ",robot.limit.getSignalValue());
        feed.add("Pivot Angle: ",robot.pivot.getAngle());
        feed.update();*/

    @Override
    public void run() throws InterruptedException {
        robot.initHardware(hardwareMap);

        while(!opModeIsActive()){
            feed.add("Limit:",robot.limit.getSignalValue());
            feed.add("Arm Angle:", robot.pivot.getAngle());
            feed.add("Yaw:", robot.imu.getYaw());
            feed.update();
        }

        waitForStart();

        while(opModeIsActive()){
            robot.driveTeleop(controller1);
            intakeTeleop(controller1);
            pivotTeleop(controller1);
            liftTeleop(controller1);
            hangTeleop(controller1);

            if(controller1.leftJoystickButtonToggle() && controller1.rightJoystickButtonToggle()){
                feed.add("SLOW MODE");
                feed.addBlankLine();
                feed.add("DT:", robot.dt.getPower());
                feed.add("Pivot Angle:",robot.pivot.getAngle());
                feed.add("Extension:",robot.lift.getDistance());
                feed.update();
            }
            else{
                feed.add("NORMAL");
                feed.addBlankLine();
                feed.add("DT:", robot.dt.getPower());
                feed.add("Pivot Angle:",robot.pivot.getAngle());
                feed.add("Extension:",robot.lift.getDistance());
                feed.add("Intake Power:",robot.intake.getPower());
                feed.update();
            }
            if(controller1.y()){
                robot.intake.setState(MaelCollector.State.INTAKE);
                robot.lift.setState(MaelElevator.State.EXTEND);
                robot.pivot.setState(MaelPivot.State.UP);
            }
            else{
                robot.intake.setState(MaelCollector.State.STOP);
                robot.lift.setState(MaelElevator.State.STOP);
                robot.pivot.setState(MaelPivot.State.STOP);
            }
            if(robot.limit.pressed()){
                robot.pivot.reset();
            }
        }
    }

    @Override
    public void initHardware() {

    }

    public void intakeTeleop(MaelController c){
        if(c.leftBumper()){
            robot.intake.setState(MaelCollector.State.INTAKE);
        }
        else if(c.leftTriggerPressed()){
            robot.intake.setState(MaelCollector.State.OUTTAKE);
        }
        else robot.intake.setState(MaelCollector.State.STOP);

        if(c.a()) robot.index.setPos(.15);
        else robot.index.setPos(.95);
    }

    public void pivotTeleop(MaelController c){
        if(c.dPadUp()) robot.pivot.setState(MaelPivot.State.UP);
        else if(c.dPadDown()) robot.pivot.setState(MaelPivot.State.DOWN);
        else robot.pivot.setState(MaelPivot.State.STOP);
    }

    public void liftTeleop(MaelController c){
        if(c.rightTriggerPressed()) robot.lift.setState(MaelElevator.State.EXTEND);
        else if(c.rightBumper()) robot.lift.setState(MaelElevator.State.RETRACT);
        else robot.lift.setState(MaelElevator.State.STOP);
    }

    public void hangTeleop(MaelController c){
        if(c.x()) robot.hang.setPower(.85);
        else if(c.b()) robot.hang.setPower(-.85);
        else robot.hang.setPower(0);
    }


}
