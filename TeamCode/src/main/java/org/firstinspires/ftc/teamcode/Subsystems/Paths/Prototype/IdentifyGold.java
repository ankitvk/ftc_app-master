package org.firstinspires.ftc.teamcode.Subsystems.Paths.Prototype;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;

public class IdentifyGold {

    Hardware hardware;
    Telemetry telemetry;
    private GoldFind goldfish;

    Positions gold;

    public IdentifyGold(Hardware hardware,GoldFind goldfish){
        this.hardware = hardware;
        this.telemetry = hardware.telemetry;
        this.goldfish = goldfish;
    }

    public Positions identify(){
        if(!goldfish.detector.isFound()){
            gold = Positions.RIGHT;
            telemetry.addLine("RIGHT");
            telemetry.update();
        }
        else if(goldfish.getXPosition()>35){
            gold = Positions.LEFT;
            telemetry.addLine("LEFT");
            telemetry.update();
        }
        else if(goldfish.getXPosition()<=35){
            gold = Positions.MIDDLE;
            telemetry.addLine("MIDDLE");
            telemetry.update();
        }
        return gold;
    }

    public enum Positions{
        LEFT,
        MIDDLE,
        RIGHT
    }

    public double returnX(){
        return goldfish.getXPosition();
    }
}
