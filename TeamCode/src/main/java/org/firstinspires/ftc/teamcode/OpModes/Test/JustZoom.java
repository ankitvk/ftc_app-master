package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Disabled
@Autonomous(name = "JustZoom",group = "Dummy")
public class JustZoom extends LinearOpMode implements AutonomousOpMode,Constants{

    Hardware robot = new Hardware();
    private double speedMultiplier = 1;
    private double speedMultiplier2 = 1;
    PIDController control1 = new PIDController(0.0165,10,0,1);
    PIDController control2 = new PIDController(0.01,0,0,1);

    public boolean getOpModeIsActive() {
        return opModeIsActive();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    @Override
    public void runOpMode() {
        robot.setAuto(this, telemetry);

        robot.init(hardwareMap);

        telemetry.addLine("Instant Run test 3");
        telemetry.addData("angle ", robot.getImu().getRelativeYaw());
        telemetry.update();

        waitForStart();

        double angle = robot.getImu().getRelativeYaw();

        while(opModeIsActive() && (Math.abs(angle-90)>=1)){


            speedMultiplier = control1.power(90,angle);
            telemetry.addData("Power: ", speedMultiplier);
            telemetry.addData("leftDrive: ", robot.getFrontLeft().getPower());
            telemetry.addData("rightDrive: ", robot.getFrontRight().getPower());
            telemetry.addData("Angle: ", robot.getImu().getYaw());
            telemetry.addData("error: ",control1.getError());
            telemetry.addData("KP*error: ",control1.returnVal()[0]);
            telemetry.addData("KI*i: ",control1.returnVal()[1]);
            telemetry.update();

            robot.getFrontLeft().setPower(speedMultiplier*.225);
            robot.getBackLeft().setPower(speedMultiplier*.225);
            robot.getFrontRight().setPower(speedMultiplier);
            robot.getBackRight().setPower(speedMultiplier);

            angle = robot.getImu().getRelativeYaw();
        }

        robot.getDrivetrain().driveForwardDistance(20);

        while(opModeIsActive() && (Math.abs(angle-133)>=3)){

            speedMultiplier = control1.power(90,angle);
            telemetry.addData("Power: ", speedMultiplier);
            telemetry.addData("leftDrive: ", robot.getFrontLeft().getPower());
            telemetry.addData("rightDrive: ", robot.getFrontRight().getPower());
            telemetry.addData("Angle: ", robot.getImu().getYaw());
            telemetry.addData("error: ",control1.getError());
            telemetry.addData("KP*error: ",control1.returnVal()[0]);
            telemetry.addData("KI*i: ",control1.returnVal()[1]);
            telemetry.update();

            robot.getFrontLeft().setPower(speedMultiplier*.225);
            robot.getBackLeft().setPower(speedMultiplier*.225);
            robot.getFrontRight().setPower(speedMultiplier);
            robot.getBackRight().setPower(speedMultiplier);

            angle = robot.getImu().getRelativeYaw();

        }

        robot.getDrivetrain().driveForwardDistance(20);

        sleep(500);

        robot.getDrivetrain().driveForwardDistance(-20);

        while(opModeIsActive() && (Math.abs(angle-87)>=3)){


            robot.getFrontLeft().setPower(-speedMultiplier*.225);
            robot.getBackLeft().setPower(-speedMultiplier*.225);
            robot.getFrontRight().setPower(-speedMultiplier);
            robot.getBackRight().setPower(-speedMultiplier);

            angle = robot.getImu().getRelativeYaw();

            telemetry.addData("leftDrive: ", robot.getFrontLeft().getPower());
            telemetry.addData("rightDrive: ", robot.getFrontRight().getPower());
            telemetry.addData("angle ", robot.getImu().getRelativeYaw());
            telemetry.update();

        }

        robot.getDrivetrain().driveForwardDistance(-15);

        while(opModeIsActive() && (Math.abs(angle)>=2)){


            robot.getFrontLeft().setPower(-speedMultiplier*.225);
            robot.getBackLeft().setPower(-speedMultiplier*.225);
            robot.getFrontRight().setPower(-speedMultiplier);
            robot.getBackRight().setPower(-speedMultiplier);

            angle = robot.getImu().getRelativeYaw();

            telemetry.addData("leftDrive: ", robot.getFrontLeft().getPower());
            telemetry.addData("rightDrive: ", robot.getFrontRight().getPower());
            telemetry.addData("angle ", robot.getImu().getRelativeYaw());
            telemetry.update();

        }
    }

    private double distanceToTicks(double distance){
        return (distance/(Companion.getWHEEL_DIAMETER() *Math.PI))* Companion.getDT_GEARBOX_TICKS_PER_ROTATION();
    }

    private double ticksToDistance(double ticks){
        return (ticks*(Companion.getWHEEL_DIAMETER() *Math.PI))/ Companion.getDT_GEARBOX_TICKS_PER_ROTATION();
    }
}
