package org.firstinspires.ftc.teamcode.OpModes.Test;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Drivers.LEDRiver;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@TeleOp(name = "LEDRiver Demo")
public class LEDTest extends LinearOpMode implements Constants {

    public Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        LynxModule revHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        try {
            new LynxI2cConfigureChannelCommand(revHub, 1, LynxI2cConfigureChannelCommand.SpeedCode.FAST_400K).send();
        } catch (LynxNackException | InterruptedException ex) {
            ex.printStackTrace();
        }

        LEDRiver ledRiver = hardwareMap.get(LEDRiver.IMPL, "ledriver");
        ledRiver.setLEDCount(Companion.getLedCount());
        ledRiver.setMode(LEDRiver.Mode.SOLID);
        ledRiver.setLEDMode(LEDRiver.LEDMode.RGB);
        ledRiver.setColorDepth(LEDRiver.ColorDepth.BIT_16);

        //ledRiver.setColor(new LEDRiver.Color(23, 76, 249, 225));


        //ledRiver.setColor(0, new LEDRiver.Color(23, 76, 249,0));
        //ledRiver.setColor(1, Color.WHITE);

        waitForStart();

        fade(ledRiver, new LEDRiver.Color(225, 225, 225, 0), new LEDRiver.Color(23, 76, 249, 0), 15);

/*        while(opModeIsActive()){
            double x = System.nanoTime()/1e6;
            int time = (int) Math.round(x);

            for (int i = 0; i < ledCount; i+=2) {
                if (i <= ledCount - 6) {
                    ledRiver.setColor(i, new LEDRiver.Color(23, 76, 249, time));
                    ledRiver.setColor(i + 1, new LEDRiver.Color(23, 76, 249, time));
                    ledRiver.apply();

                }
                for (int j = i + 2; j <= i + 6; j++) {
                    if (j < ledCount) {
                        ledRiver.setColor(j, Color.WHITE);
                    }
                }
                    //ledRiver.apply();
                    try{
                        Thread.sleep(1);
                    } catch(InterruptedException e) {

                    }
            }

        }*/


        while (opModeIsActive()) {
            /*double x = System.nanoTime()/1e6;
            int time = (int) Math.round(x);

            for (int i = 0; i < ledCount; i+=2) {
                    //ledRiver.setColor(i, new LEDRiver.Color(23, 76, 249, time));
                    ledRiver.setColor(i, new LEDRiver.Color(255, 0, 0, 0));
                    ledRiver.setColor(i + 1, new LEDRiver.Color(23, 76, 249, time));
                    ledRiver.apply();*/


/*            ledRiver.setColor(150, new LEDRiver.Color(23, 76, 249, time));
            ledRiver.apply();*/

/*            for (int i = 0; i < ledCount; i+=2) {
                if (i <= ledCount - 6) {
                    ledRiver.setColor(i, new LEDRiver.Color(23, 76, 249, time));
                    ledRiver.setColor(i + 1, new LEDRiver.Color(23, 76, 249, time));
                    ledRiver.apply();

                }
                for (int j = i + 2; j <= i + 6; j++) {
                    if (j < ledCount) {
                        ledRiver.setColor(j, Color.WHITE);
                    }
                }
                //ledRiver.apply();
                try{
                    Thread.sleep(1);
                } catch(InterruptedException e) {

                }
            }*/

        }

/*        ledRiver.setPattern(LEDRiver.Pattern.THEATRE_RUNNING.builder());
        ledRiver.apply();*/
        //setTimePattern();
/*        while (opModeIsActive()) {
            for (int i = 0; i < ledCount; i+=2) {
                if (i <= ledCount - 6) {
                    ledRiver.setColor(i, new LEDRiver.Color(23, 76, 249, 0));
                    ledRiver.setColor(i + 1, new LEDRiver.Color(23, 76, 249, 0));
                }
                for (int j = i + 2; j <= i + 6; j++) {
                    if (j < ledCount) {
                        ledRiver.setColor(j, Color.WHITE);
                    }
                }
                ledRiver.apply();
                try{
                    Thread.sleep(1);
                } catch(InterruptedException e) {

                }
            }

            for (int i = ledCount - 1; i >= 0; i-=2) {
                if (i > 4) {
                    ledRiver.setColor(i, new LEDRiver.Color(23, 76, 249, 0));
                    ledRiver.setColor(i - 1, new LEDRiver.Color(23, 76, 249, 0));
                }
                for (int j = i - 2; j >= i - 6; j--) {
                    if (j >= 0) {
                        ledRiver.setColor(j, Color.WHITE);
                    }
                }
                ledRiver.apply();
                try{
                    Thread.sleep(1);
                } catch(InterruptedException e) {

                }
            }*/
    }


    public void setTimePattern() {
        while (opModeIsActive()) {
            double x = System.nanoTime() / 1e6;
            int time = (int) Math.round(x);

            for (int i = 0; i < Companion.getLedCount(); i += 2) {
                if (i <= Companion.getLedCount() - 6) {
                    robot.getLedRiver().setColor(i, new LEDRiver.Color(23, 76, 249, time));
                    robot.getLedRiver().setColor(i + 1, new LEDRiver.Color(23, 76, 249, time));
                }
                /*for (int j = i + 2; j <= i + 6; j++) {
                    if (j < ledCount) {
                        robot.ledRiver.setColor(j, Color.WHITE);
                    }
                }*/
                /*    robot.ledRiver.apply();
                    try{
                        Thread.sleep(1);
                    } catch(InterruptedException e) {

                    }*/
            }

        }
    }

    public void setMaelstromPattern() {
        while (opModeIsActive()) {
            for (int i = 0; i < Companion.getLedCount(); i += 2) {
                if (i <= Companion.getLedCount() - 6) {
                    robot.getLedRiver().setColor(i, new LEDRiver.Color(23, 76, 249, 0));
                    robot.getLedRiver().setColor(i + 1, new LEDRiver.Color(23, 76, 249, 0));
                }
                for (int j = i + 2; j <= i + 6; j++) {
                    if (j < Companion.getLedCount()) {
                        robot.getLedRiver().setColor(j, Color.WHITE);
                    }
                }
                robot.getLedRiver().apply();
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {

                }
            }

            for (int i = Companion.getLedCount() - 1; i >= 0; i -= 2) {
                if (i > 4) {
                    robot.getLedRiver().setColor(i, new LEDRiver.Color(23, 76, 249, 0));
                    robot.getLedRiver().setColor(i - 1, new LEDRiver.Color(23, 76, 249, 0));
                }
                for (int j = i - 2; j >= i - 6; j--) {
                    if (j >= 0) {
                        robot.getLedRiver().setColor(j, Color.WHITE);
                    }
                }
                robot.getLedRiver().apply();
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {

                }
            }
        }
    }

    private void fade(LEDRiver led, LEDRiver.Color current, LEDRiver.Color desire, double time) {
        int currentVal[] = {current.getR(), current.getB(), current.getW()};
        int desireVal[] = {desire.getR(), desire.getB(), desire.getW()};

        double colorVelocity[] = {(desireVal[0] - currentVal[0]) / time, (desireVal[1] - currentVal[1]) / time, (desireVal[2] - currentVal[2]) / time};

        while (opModeIsActive() && currentVal != desireVal) {
            led.setColor(new LEDRiver.Color(currentVal[0] + (int) colorVelocity[0] * (int) System.nanoTime() / 1000, currentVal[1] + (int) colorVelocity[1] * (int) System.nanoTime() / 1000, currentVal[2] + (int) colorVelocity[2] * (int) System.nanoTime() / 1000, 0));
            led.apply();
            telemetry.addData("Red: ",current.getG());
            telemetry.addData("Green: :",current.getG());
            telemetry.addData("Blue:",current.getB());
            telemetry.addData("White:",current.getW());
            telemetry.update();

        }

/*        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {

        }*/


    }
}