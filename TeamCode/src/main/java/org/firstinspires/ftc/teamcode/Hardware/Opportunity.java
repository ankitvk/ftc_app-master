package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Control.Constants;

import ftc.library.MaelMotions.MaelMotors.Motor;
import ftc.library.MaelRobot;
import ftc.library.MaelSensors.MaelIMU;
import ftc.library.MaelSubsystems.MaelstromDrivetrain.DrivetrainModels;
import ftc.library.MaelSubsystems.MaelstromDrivetrain.MaelDrivetrain;

public class Opportunity extends MaelRobot implements Constants {

    @Override
    public void initHardware(HardwareMap hwMap) {
        dt = new MaelDrivetrain(DrivetrainModels.MECH_ROBOT,mecanum_ratio,dtKP,dtKI,dtKD,hwMap, Motor.ORBITAL20);
        dt.setClosedLoop(false);
        setSpeedMultiplier(1);
        imu = new MaelIMU("imu", hwMap);
    }
}
