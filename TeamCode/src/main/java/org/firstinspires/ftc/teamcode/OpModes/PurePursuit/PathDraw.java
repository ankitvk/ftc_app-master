package org.firstinspires.ftc.teamcode.OpModes.PurePursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelControl.PurePursuit.warriorlib.Path;
import ftc.library.MaelUtils.MaelUtils;
import ftc.library.MaelWrappers.MaelLinearOp;

@Autonomous(name = "Drawing a Path")
public class PathDraw extends MaelLinearOp {
    private ArrayList<MaelPose> list = new ArrayList<>();
    private Path drawPath = new Path();
    private FtcDashboard dash = MaelUtils.dashboard;
    MaelPose point1 = new MaelPose(2,5);
    MaelPose point2 = new MaelPose(4,10);
    MaelPose point3 = new MaelPose(5,90);
    TelemetryPacket packet = new TelemetryPacket();
    double[] xPoints = {point1.x,point2.x,point3.x};
    double[] yPoints = {point1.y,point2.y,point3.y};

    @Override
    public void run() throws InterruptedException {

        waitForStart();

        packet.put("Point 1 X: ",point1.x);
        packet.put("Point 1 Y: ",point1.y);
        packet.put("Point 2 X: ",point2.x);
        packet.put("Point 2 Y: ",point2.y);
        packet.put("Point 3 X: ",point3.x);
        packet.put("Point 3 Y: ",point3.y);
        packet.fieldOverlay()
                .strokePolyline(xPoints,yPoints);
        dash.sendTelemetryPacket(packet);

    }

    @Override
    public void initHardware() {

        drawPath.addPoints(point1,point2);
    }
}
