package org.firstinspires.ftc.teamcode.Subsystems.Paths.Prototype;

import org.firstinspires.ftc.teamcode.Drivers.LEDRiver;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class TeleopLED {

    Hardware hardware;
    LEDRiver ledRiver;

    public TeleopLED(Hardware hardware){
        this.hardware = hardware;
        this.ledRiver = hardware.getLedRiver();

    }

    public void run(long time){
        double timeInSec = time/1000000000;
        if(timeInSec >=60 && timeInSec <62 ){
            ledRiver.setColor(new LEDRiver.Color(23, 76, 249, 225));
            ledRiver.setPattern(LEDRiver.Pattern.STROBE.builder());
        }
        else if(timeInSec >=90 && timeInSec <92){
            ledRiver.setColor(new LEDRiver.Color(23, 76, 249, 225));
            ledRiver.setPattern(LEDRiver.Pattern.STROBE.builder());
        }
        else if(timeInSec >=100 && timeInSec <102){
            ledRiver.setColor(new LEDRiver.Color(23, 76, 249, 225));
            ledRiver.setPattern(LEDRiver.Pattern.STROBE.builder());
        }
        else if (timeInSec >= 118){
            ledRiver.setMode(LEDRiver.Mode.SOLID);
            ledRiver.setColor(new LEDRiver.Color(23, 76, 249, 225));
        }
        else{
            ledRiver.setColor(new LEDRiver.Color(23, 76, 249, 225));
            ledRiver.setPattern(LEDRiver.Pattern.BREATHING.builder());
        }
        ledRiver.apply();
    }
}
