package org.firstinspires.ftc.teamcode.Subsystems.Components;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.Toggle;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class Endgame implements Constants {

     Hardware hardware;
     Toggle swivelToggle = new Toggle();

     public Endgame(Hardware hardware){
         this.hardware = hardware;
     }

     public void hookSwivel(Gamepad gamepad){

         if(gamepad.right_bumper){
             hardware.hookSwivel.setPosition(.75);
         }
         else if(gamepad.left_bumper){
             hardware.hookSwivel.setPosition(.45);
         }
     }

     public void hookRelease (Gamepad gamepad){
         if (gamepad.y){
             hardware.hookRelease.setPosition(1);
         }
         else if (gamepad.x){
             hardware.hookRelease.setPosition(0);
         }
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

     public void drop (Gamepad gamepad){
         if (gamepad.dpad_down){
             hardware.drop.setPosition(0);
         }
         else{
             hardware.drop.setPosition(1);
         }
     }
}
