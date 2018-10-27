package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

public class PixyCam {
    private I2cDeviceSynch pixyCam;
    public PixyCam(HardwareMap hardwareMap, String name) {
        pixyCam = hardwareMap.i2cDeviceSynch.get(name);
    }

    public void initializePixy() {
        //pixyCam.setI2cAddress(I2cAddr.create7bit(/*PUT ADDRESS OF PIXYCAM FOUND THROUGH PIXYMON*/));
        pixyCam.setReadWindow(new I2cDeviceSynch.ReadWindow(1, 26, I2cDeviceSynch.ReadMode.REPEAT));
    }
}
