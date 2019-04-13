package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Control.Constants;

import ftc.library.MaelstromControl.MaelstromPID.PIDPackage;
import ftc.library.MaelstromMotions.MaelstromMotors.MotorModel;
import ftc.library.MaelstromRobot;
import ftc.library.MaelstromSensors.MaelstromIMU;
import ftc.library.MaelstromSubsystems.MaelstromDrivetrain.DrivetrainModels;
import ftc.library.MaelstromSubsystems.MaelstromDrivetrain.MaelstromDrivetrain;

public class Opportunity extends MaelstromRobot implements Constants {

    @Override
    public void initHardware(HardwareMap hwMap) {
        dt = new MaelstromDrivetrain(DrivetrainModels.MECH_ROBOT,mecanum_ratio,dtKP,dtKI,dtKD,hwMap, MotorModel.ORBITAL20);
        dt.setClosedLoop(false);
        setSpeedMultiplier(1);
        imu = new MaelstromIMU("imu", hwMap);
    }

    @Override
    public PIDPackage pidPackage() {
        return null;
    }
}
