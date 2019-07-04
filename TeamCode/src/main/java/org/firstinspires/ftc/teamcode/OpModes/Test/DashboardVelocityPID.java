package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Components.Drivetrain;

import ftc.library.MaelUtils.MaelUtils;

@Config
@Autonomous(name = "Dashboard Velocity Tuning")
public class DashboardVelocityPID extends LinearOpMode implements Constants {
    private Hardware robot =  new Hardware();
    private Drivetrain dt = new Drivetrain(robot);
    private PIDCoefficients MOTOR_PID = new PIDCoefficients();

    FtcDashboard dashboard = MaelUtils.dashboard;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.opportunityInit(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry,dashboard.getTelemetry());

        PIDCoefficients currentCoeffs = dt.backLeft.getPIDCoeffs();
        pidCopy(currentCoeffs,MOTOR_PID);
        dashboard.updateConfig();

        RobotLog.i("Initial motor PID coefficients: " + MOTOR_PID);

        NanoClock clock =  NanoClock.system();

        telemetry.log().add("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

    }

    private static void pidCopy(PIDCoefficients source, PIDCoefficients dest) {
        dest.kP = source.kP;
        dest.kI = source.kI;
        dest.kD = source.kD;
    }
}
