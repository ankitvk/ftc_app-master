package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Endgame;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Extendo;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Pivot;
@Disabled
@TeleOp(name = "servo boi")
public class ServoTest extends LinearOpMode {

    Servo left, right;

    public void runOpMode() throws InterruptedException{

        left = hardwareMap.servo.get("left");
        right = hardwareMap.servo.get("right");

        telemetry.update();

        // wait for start button.

        left.setPosition(-.25);
        right.setPosition(1.25);

        waitForStart();

        while(true){
            if (gamepad1.a){
                right.setPosition(0);
                left.setPosition(1);
            }
        }

    }
}
