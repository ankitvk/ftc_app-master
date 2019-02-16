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
public class LEDTest extends LinearOpMode {
    public int ledCount = 150;

    @Override
    public void runOpMode() throws InterruptedException {
        LynxModule revHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        try {
            new LynxI2cConfigureChannelCommand(revHub, 1, LynxI2cConfigureChannelCommand.SpeedCode.FAST_400K).send();
        } catch (LynxNackException | InterruptedException ex) {
            ex.printStackTrace();
        }

        LEDRiver ledRiver = hardwareMap.get(LEDRiver.IMPL, "ledriver");
        ledRiver.setLEDCount(ledCount);
        ledRiver.setMode(LEDRiver.Mode.INDIVIDUAL);
        ledRiver.setLEDMode(LEDRiver.LEDMode.RGB);
        ledRiver.setColorDepth(LEDRiver.ColorDepth.BIT_16);


        //ledRiver.setColor(0, new LEDRiver.Color(23, 76, 249,0));
        //ledRiver.setColor(1, Color.WHITE);

        waitForStart();

/*        ledRiver.setPattern(LEDRiver.Pattern.THEATRE_RUNNING.builder());
        ledRiver.apply();*/

        while (opModeIsActive()) {
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
            }
        }


    }
}