package org.firstinspires.ftc.teamcode.Subsystems.Components;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Control.Toggle;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFind;

public class Endgame implements Constants {

     Hardware hardware;
    private AutonomousOpMode auto;
    private Telemetry telemetry;
    private GoldFind goldfish;

     public Endgame(Hardware hardware){

         this.hardware = hardware;
         auto = hardware.auto;
         telemetry = hardware.telemetry;
     }


     public void winch (Gamepad gamepad){
         if (gamepad.right_trigger>.75){
             hardware.winch.setPower(1);
         }
         else if(gamepad.left_trigger>.75){
             hardware.winch.setPower(-1);
         }
         else{
             hardware.winch.setPower(0);
         }
     }

     public void lift(){
         boolean idc = true;
         double position = 0;
         double current = hardware.winch.getCurrentPosition();
         while(opModeIsActive()&& hardware.winch.getCurrentPosition()<current+22000){
             hardware.winch.setPower(-1);
             telemetry.addData("encoderticks: ",hardware.winch.getCurrentPosition());

             telemetry.update();
         }
         //return position;
     }
    public boolean opModeIsActive() {
        return auto.getOpModeIsActive();
    }


}
