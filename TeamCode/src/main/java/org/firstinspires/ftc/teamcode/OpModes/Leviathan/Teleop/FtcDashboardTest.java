package org.firstinspires.ftc.teamcode.OpModes.Leviathan.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.Control.Constants;

@Autonomous(name = "Testing the Dashboard",group = "Dashboard")
public class FtcDashboardTest extends LinearOpMode implements Constants {
    private FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()

                .setFill("red")
                .fillRect(-20,-20,40,40)
                .strokeLine(-20,-20,-50,-50);
        packet.put("Epic gamer moment: ", 3 + 5);
        dash.sendTelemetryPacket(packet);
    }
}
